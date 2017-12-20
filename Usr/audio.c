#include <string.h>
#include "audio.h"
#include "ff.h"
#include "freertos.h"
#include "wm8978.h"
#include "i2s.h"
#include "fatfs.h"
#include "rng.h"
#include "gpio.h"
#include "rtc.h"
#include "main.h"

osThreadId audioplayTaskHandle;
osThreadId audiocontrollerHandle;
extern EventGroupHandle_t xEventGroup;
extern QueueHandle_t xQueueLog;
extern SemaphoreHandle_t xSdioMutex;
//音乐播放控制器
__audiodev audiodev;

//*********************************变量声明*****************************
uint32_t wavsize;  //wav数据大小

uint8_t wavrxtxflag=0; //txrx传输状态标识

//**********************************************************************

//采样率计算公式:Fs=I2SxCLK/[256*(2*I2SDIV+ODD)]
//I2SxCLK=(HSE/pllm)*PLLI2SN/PLLI2SR
//一般HSE=8Mhz
//pllm:在Sys_Clock_Set设置的时候确定，一般是8
//PLLI2SN:一般是192~432
//PLLI2SR:2~7
//I2SDIV:2~255
//ODD:0/1
//I2S分频系数表@pllm=8,HSE=8Mhz,即vco输入频率为1Mhz
//表格式:采样率/10,PLLI2SN,PLLI2SR,I2SDIV,ODD
const uint16_t I2S_PSC_TBL[][5]=
{
	{800 ,256,5,12,1},		//8Khz采样率
	{1102,429,4,19,0},		//11.025Khz采样率
	{1600,213,2,13,0},		//16Khz采样率
	{2205,429,4, 9,1},		//22.05Khz采样率
	{3200,213,2, 6,1},		//32Khz采样率
	{4410,271,2, 6,0},		//44.1Khz采样率
	{4800,258,3, 3,1},		//48Khz采样率
	{8820,316,2, 3,1},		//88.2Khz采样率
	{9600,344,2, 3,1},  	//96Khz采样率
	{17640,361,2,2,0},  	//176.4Khz采样率
	{19200,393,2,2,0},  	//192Khz采样率
};
//1使能蓝牙模块
static void BT_Power(uint8_t enable)
{
#if defined(F429_ZET6)
	if (enable)
	{
		HAL_GPIO_WritePin(RESET_BT_GPIO_Port,RESET_BT_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MODE_BT_GPIO_Port,MODE_BT_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(REF_EN_GPIO_Port,REF_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PAIR_BT_GPIO_Port,PAIR_BT_Pin,GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(PAIR_BT_GPIO_Port,PAIR_BT_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(REF_EN_GPIO_Port,REF_EN_Pin,GPIO_PIN_RESET);
	}
#endif
}
static void send_log(char *log)
{
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
	BaseType_t xStatus;
	char queuebuffer[QUEUE_LOG_ITEM_SIZE];

	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);
	sprintf(queuebuffer,"%04d/%02d/%02d-%02d/%02d/%02d/%03d    %s\n",\
		dat.Year+2000,dat.Month,dat.Date,\
		tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds,\
		log);
	xStatus = xQueueSend(xQueueLog,queuebuffer,portMAX_DELAY);
	if (xStatus != pdPASS)
	{
		app_trace_log("Could not send to the queue.\n");
	}
}
static FRESULT wav_decode_init(uint8_t *fname, __wavctrl* wavx)
{
	FIL *ftemp;
	FRESULT res = FR_OK;
	uint8_t *buf;
	uint32_t br = 0;
	char path[40];

	ChunkRIFF *riff;
	ChunkFMT *fmt;
	ChunkFACT *fact;
	ChunkDATA *data;

	ftemp = (FIL*)pvPortMalloc(sizeof(FIL));
	buf = pvPortMalloc(512);
	if(!ftemp || !buf) //内存申请成功
	{
		goto error2;
	}
	app_trace_log("malloc success!\n");

	strcpy(path,MUSIC_PATH);
	strcat(path,(const TCHAR*)fname);
	app_trace_log(path);
	app_trace_log("\n");
	res = f_open(ftemp,(const TCHAR*)path,FA_READ);
	if(res != FR_OK)
	{
        app_trace_log("error %d %d %s\n",res,__LINE__,__FUNCTION__);
		goto error2;
	}
	res = f_read(ftemp,buf,512,&br); //读取512字节数据
    if (res != FR_OK)
    {
        app_trace_log("error %d %d %s\n",res,__LINE__,__FUNCTION__);
    	goto error1;
    }
    riff=(ChunkRIFF*)buf;  //获取RIFF块
    if(riff->Format == 0x45564157)//是WAV文件"WAVE"
    {
        fmt=(ChunkFMT *)(buf + 12);
        fact=(ChunkFACT *)(buf + 12 + 8 + fmt->ChunkSize);//读取FACT块
        if(fact->ChunkID==0x74636166||fact->ChunkID==0x5453494C)
            wavx->datastart=12+8+fmt->ChunkSize+8+fact->ChunkSize;
        else
            wavx->datastart=12+8+fmt->ChunkSize;
        data=(ChunkDATA*)(buf+wavx->datastart); //读取DATA块
        if(data->ChunkID==0x61746164)//解析成功
        {
            wavx->audioformat=fmt->AudioFormat;		//音频格式
            wavx->nchannels=fmt->NumOfChannels;		//通道数
            wavx->samplerate=fmt->SampleRate;		//采样率
            wavx->bitrate=fmt->ByteRate*8;			//得到位速
            wavx->blockalign=fmt->BlockAlign;		//块对齐
            wavx->bps=fmt->BitsPerSample;			//位数,16/24/32位

            wavx->datasize=data->ChunkSize;			//数据块大小
            wavx->datastart=wavx->datastart+8;		//数据流开始的地方.

            app_trace_log("wavx->audioformat:%d\r\n",wavx->audioformat);
            app_trace_log("wavx->nchannels:%d\r\n",wavx->nchannels);
            app_trace_log("wavx->samplerate:%d\r\n",wavx->samplerate);
            app_trace_log("wavx->bitrate:%d\r\n",wavx->bitrate);
            app_trace_log("wavx->blockalign:%d\r\n",wavx->blockalign);
            app_trace_log("wavx->bps:%d\r\n",wavx->bps);
            app_trace_log("wavx->datasize:%d\r\n",wavx->datasize);
            app_trace_log("wavx->datastart:%d\r\n",wavx->datastart);
        }
    }
error1:
	res = f_close(ftemp);
    if(res !=FR_OK)
    {
        app_trace_log("error %d %d %s\n",res,__LINE__,__FUNCTION__);
    }
error2:
	vPortFree(ftemp);
	vPortFree(buf);

    return res;
}

//填充buf
//buf:数据区
//tbuf:临时数据区
//size:填充数据量
//bits:位数(16/24)
//返回值:读到的数据个数
static uint32_t wav_buffill(uint8_t *buf,uint8_t *tbuf,FIL*file,uint16_t size,uint8_t bits)
{
	uint16_t readlen=0;
	uint32_t bread = 0;
	uint16_t i = 0;
	uint8_t *p;
    FRESULT res;

	xSemaphoreTake(xSdioMutex,portMAX_DELAY);
	if(bits==24)//24bit音频,需要处理一下
	{
		readlen=(size/4)*3;							//此次要读取的字节数
		res = f_read(file,tbuf,readlen,(UINT*)&bread);	//读取数据
		if (res != FR_OK)
		{
			return 0;
		}
		p=audiodev.tbuf;
		for(i=0;i<size;)
		{
			buf[i++]=p[1];
			buf[i]=p[2];
			i+=2;
			buf[i++]=p[0];
			p+=3;
		}
		bread=(bread*4)/3;		//填充后的大小.
	}
	else
	{
		res = f_read(file,buf,size,(UINT*)&bread);//16bit音频,直接读取数据
        if (res == FR_OK)
        {
            if(bread<size)//不够数据了,补充0
            {
                for(i=bread;i<size-bread;i++)buf[i]=0;
            }
        }
		else
		{
			APP_ERROR_CHECK(res);
		}
	}
	xSemaphoreGive(xSdioMutex);
	return bread;
}

//得到当前播放时间
//fx:文件指针
//wavx:wav播放控制器
//static void wav_get_curtime(FIL*fx,__wavctrl *wavx)
//{
//	long long fpos;
// 	wavx->totsec=wavx->datasize/(wavx->bitrate/8);	//歌曲总长度(单位:秒)
//	fpos=fx->fptr-wavx->datastart; 					//得到当前文件播放到的地方
//	wavx->cursec=fpos*wavx->totsec/wavx->datasize;	//当前播放到第多少秒了?
//}

static void IIS_Init(uint32_t DataFormat,uint32_t AudioFreq)
{
    uint8_t i = 0;
    uint32_t samplerate = AudioFreq / 10;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    for (i= 0; i < (sizeof(I2S_PSC_TBL)/10);i++)
    {
        if(samplerate==I2S_PSC_TBL[i][0])
            break;
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = I2S_PSC_TBL[i][1];
    PeriphClkInitStruct.PLLI2S.PLLI2SR = I2S_PSC_TBL[i][2];

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }


	if(DataFormat==8||DataFormat==16)
	{
		hi2s2.Init.DataFormat=I2S_DATAFORMAT_16B;
	}
	else
	if(DataFormat==24)
	{
		hi2s2.Init.DataFormat=I2S_DATAFORMAT_24B;
	}
	else
	if(DataFormat==32)
	{
		hi2s2.Init.DataFormat=I2S_DATAFORMAT_32B;
	}
	else
		return ;//not support
    hi2s2.Init.AudioFreq = AudioFreq;
#if defined(IIS_MASTER_TX)
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
#else
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
#endif
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK)
    {
        Error_Handler();
    }
}

//通过时间获取文件名
//仅限在SD卡保存,不支持FLASH DISK保存
//组合成:形如"0:RECORDER/REC20120321210633.wav"的文件名
static void recoder_new_pathname(uint8_t *pname)
{
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);

	sprintf((char*)pname,"%04d-%02d-%02d-%02d-%02d-%02d.wav",\
		 dat.Year+2000,dat.Month,dat.Date,tim.Hours,tim.Minutes,tim.Seconds);
}

//初始化WAV头.
void recoder_wav_init(__WaveHeader* wavhead,uint32_t DataFormat,uint32_t AudioFreq) //初始化WAV头
{
	wavhead->riff.ChunkID=0X46464952;	//"RIFF"
	wavhead->riff.ChunkSize=0;			//还未确定,最后需要计算
	wavhead->riff.Format=0X45564157; 	//"WAVE"
	wavhead->fmt.ChunkID=0X20746D66; 	//"fmt "
	wavhead->fmt.ChunkSize=16; 			//大小为16个字节
	wavhead->fmt.AudioFormat=0X01; 		//0X01,表示PCM;0X01,表示IMA ADPCM
 	wavhead->fmt.NumOfChannels=2;		//双声道
 	wavhead->fmt.SampleRate=AudioFreq;		//16Khz采样率 采样速率
 	wavhead->fmt.ByteRate=wavhead->fmt.SampleRate*4;//字节速率=采样率*通道数*(ADC位数/8)
 	wavhead->fmt.BlockAlign=4;			//块大小=通道数*(ADC位数/8)
 	wavhead->fmt.BitsPerSample=DataFormat;		//16位PCM
   	wavhead->data.ChunkID=0X61746164;	//"data"
 	wavhead->data.ChunkSize=0;			//数据大小,还需要计算
}

//*******************************************************************************
//获取播放列表 返回音乐文件个数
//*******************************************************************************
static uint8_t Get_Play_List(void)
{
	DIR recdir;
	FRESULT res;
	FILINFO wavfileinfo;
	uint8_t file_count = 0;
	res = f_opendir(&recdir,"0:/MUSIC");
	if (res != FR_OK)
	{
		goto error;
	}
	while(1)
	{
		res = f_readdir(&recdir,&wavfileinfo);
		if (res != FR_OK)
		{
			break;
		}
		if (wavfileinfo.fname[0] == 0)
		{
			break;
		}
		if(strstr(wavfileinfo.fname,".wav")||strstr(wavfileinfo.fname,".WAV"))
		{
			file_count ++ ;
			app_trace_log("%s\r\n",wavfileinfo.fname);
		}
	}

error:
	f_closedir(&recdir);
	return file_count;
}

void Get_Play_Song(uint8_t *fname,uint8_t file_count)
{
	uint32_t number;

	number = HAL_RNG_GetRandomNumber(&hrng);
	app_trace_log("%s,num %uld, count %d\n",__FUNCTION__,number,file_count);
	number = number % file_count + 1;
	sprintf((char*)fname,"%03d.wav",number);
}
//*******************************************************************************
//音频控制线程
//*******************************************************************************
void AudioController_Task(void const * argument)
{
	EventBits_t xEventGroupValue;
	char log[40];
	static uint8_t usb_status = 0;
	static uint8_t ble_status = 0;//蓝牙状态，初始状态蓝牙关闭
	const EventBits_t xBitsToWaitFor = (EVENTS_VOL_UP_BIT|
		                                  EVENTS_VOL_DOWN_BIT|
		                                  EVENTS_FUN_BLE_CHANGE_BIT|
		                                  EVENTS_FUN_USB_BIT|
										  EVENTS_ASK_BIT);
//	const EventBits_t uxAllSyncBits = ( EVENTS_VOL_UP_BIT |
//										EVENTS_VOL_DOWN_BIT |
//										EVENTS_FUN_STOP_BIT |
//										EVENTS_PLAY_BIT |
//										EVENTS_RECORD_BIT |
//										EVENTS_PLAY_AND_RECORD_BIT |
//										EVENTS_PLAY_END_BIT |
//										EVENTS_RECORD_BIT |
//										EVENTS_PLAY_AND_RECORD_BIT |
//										EVENTS_ASK_BIT |
//										EVENTS_FUN_BLE_OPEN_BIT|
//										EVENTS_FUN_BLE_CLOSE_BIT);
	for(;;)
	{
		//堵塞模式
		xEventGroupValue = xEventGroupWaitBits(/* The event group to read. */
											   xEventGroup,
											   /* Bits to test. */
											   xBitsToWaitFor,
											   /* Clear bits on exit if the
																						  unblock condition is met. */
											   pdTRUE,
											   /* Don't wait for all bits. This
																						  parameter is set to pdTRUE for the
																						  second execution. */
											   pdFALSE,
											   /* Don't time out. */
											   portMAX_DELAY);
		if((xEventGroupValue&EVENTS_VOL_UP_BIT)!=0)
		{
			volume+=5;
			if (volume > 63)
			{
				volume = 63;
			}
			WM8978_HPvol_Set(volume,volume);
			WM8978_SPKvol_Set(volume);
			//记录log
			sprintf(log,"volume+");
			send_log(log);
			app_trace_log("volume+\n");
		}
		if((xEventGroupValue&EVENTS_VOL_DOWN_BIT)!=0)
		{
			volume -= 5;
			if(volume < 0)
			{
				volume = 0;
			}
			WM8978_HPvol_Set(volume,volume);
			WM8978_SPKvol_Set(volume);
			//记录log
			sprintf(log,"volume-");
			send_log(log);
			app_trace_log("volume-\n");
		}
		if((xEventGroupValue&EVENTS_FUN_BLE_CHANGE_BIT)!=0)
		{//使用蓝牙/耳机切换
		    if(ble_status)
		    {
				ble_status = 0;
				BT_Power(0);
			}
			else
			{
				ble_status = 1;
				BT_Power(1);
				sprintf(log,"BLE Pairing");
				send_log(log);
			}
		}
		if((xEventGroupValue&EVENTS_FUN_USB_BIT)!=0)
		{//打开关闭usb
			if (usb_status == 1)
			{
				HAL_GPIO_WritePin(USB_CRT_GPIO_Port,USB_CRT_Pin,GPIO_PIN_SET);
				usb_status = 0;
			}
			else
			{
				HAL_GPIO_WritePin(USB_CRT_GPIO_Port,USB_CRT_Pin,GPIO_PIN_RESET);
				usb_status = 1;
			}
		}
		if((xEventGroupValue&EVENTS_ASK_BIT)!=0)
		{
			sprintf(log,"report");
			send_log(log);
			app_trace_log("report\n");
		}
	}

}
//*******************************************************************************
//音频播放录音线程
//*******************************************************************************
void AudioPlay_Task(void const * argument)
{
	//获取music文件夹下音频文件
	FRESULT res;
	EventBits_t xEventGroupValue;
    FATFS fs;
	DIR recdir;
	//wav文件头
	__WaveHeader *wavheadrx;
	__wavctrl wavctrl;		//WAV控制结构体
	uint8_t file_count;
	//录音文件名
	uint8_t *rname;
	//播放文件名
	uint8_t *pname;
	uint32_t bw;
	uint32_t fillnum;
	uint32_t ulEventsToProcess;
	//开始播放
	uint8_t play_begin = 0;
	//开始录音
	uint8_t record_begin = 0;
	//停止音频工作
	uint8_t stop_play_record = 0;
	//完成路径
	char path[40];
	char log[40];
	const EventBits_t xBitsToWaitFor = (EVENTS_FUN_STOP_BIT |
	                                     EVENTS_PLAY_BIT |
	                                     EVENTS_RECORD_BIT |
	                                     EVENTS_PLAY_AND_RECORD_BIT);
//	const EventBits_t uxAllSyncBits = ( EVENTS_VOL_UP_BIT |
//										EVENTS_VOL_DOWN_BIT |
//										EVENTS_FUN_STOP_BIT |
//										EVENTS_PLAY_BIT |
//										EVENTS_RECORD_BIT |
//										EVENTS_PLAY_AND_RECORD_BIT |
//										EVENTS_PLAY_END_BIT |
//										EVENTS_RECORD_BIT |
//										EVENTS_PLAY_AND_RECORD_BIT |
//										EVENTS_ASK_BIT |
//										EVENTS_FUN_BLE_OPEN_BIT|
//										EVENTS_FUN_BLE_CLOSE_BIT);
	//挂载SD卡
	res = f_mount(&fs,(const TCHAR*)SD_Path,0);
	APP_ERROR_CHECK(res);
	//获取播放音乐文件
	file_count = Get_Play_List();
	//打开录音文件夹，如果没有创建
	while(f_opendir(&recdir,"0:/RECORD"))
	{
		res = f_mkdir("0:/RECORD");
		if (res != FR_OK)
		{
			app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			APP_ERROR_CHECK(res);
		}
	}
	f_closedir(&recdir);
    WM8978_Init();
	//申请播放缓存
	audiodev.file1=(FIL*)pvPortMalloc(sizeof(FIL));
	//申请录音缓存
	audiodev.file2=(FIL*)pvPortMalloc(sizeof(FIL));
	//申请播放文件handle
	audiodev.i2sbuf1=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);
	//申请录音文件handle
	audiodev.i2sbuf2=pvPortMalloc(WAV_I2S_RX_DMA_BUFSIZE);
	//用于数据填充时使用
	audiodev.tbuf=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);
	//申请录音文件头
	wavheadrx = (__WaveHeader*)pvPortMalloc(sizeof(__WaveHeader));
	//申请录音文件名
	rname = pvPortMalloc(40);
	//申请播放文件名
	pname = pvPortMalloc(40);
	if(!audiodev.file1 || !audiodev.file2 || !audiodev.i2sbuf1 || !audiodev.i2sbuf2 || !audiodev.tbuf ||\
			!wavheadrx || !wavheadrx || !rname || !pname)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		NVIC_SystemReset();
	}
	//默认录音采样率、采样位数
	wavctrl.bps = 16;
	wavctrl.samplerate = I2S_AUDIOFREQ_16K;
	for(;;)
	{	//非堵塞模式
		xEventGroupValue = xEventGroupWaitBits(/* The event group to read. */
		                                       xEventGroup,
		                                       /* Bits to test. */
		                                       xBitsToWaitFor,
		                                       /* Clear bits on exit if the
	                                                                                      unblock condition is met. */
		                                       pdTRUE,
		                                       /* Don't wait for all bits. This
	                                                                                      parameter is set to pdTRUE for the
	                                                                                      second execution. */
		                                       pdFALSE,
		                                       /* Don't time out. */
		                                       0);
		//等待事件同步
//		xEventGroupSync(xEventGroup,
//						xBitsToWaitFor,
//						uxAllSyncBits,
//						portMAX_DELAY);


		if((xEventGroupValue & EVENTS_PLAY_BIT)!=0 || (xEventGroupValue & EVENTS_PLAY_AND_RECORD_BIT)!=0)
		{//播放部分初始化
			//获取随机播放文件名
			Get_Play_Song(pname,file_count);
			//得到文件的信息
			res = wav_decode_init(pname,&wavctrl);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				stop_play_record = 1;
				goto end;
			}
			strcpy(path,MUSIC_PATH);
			strcat(path,(const char*)pname);
			//打开播放文件
			res=f_open(audiodev.file1,(const TCHAR*)path,FA_READ);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				continue;
			}
			//跳过播放文件头
			res = f_lseek(audiodev.file1, wavctrl.datastart);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error1;
			}
			//将播放信息发送给log线程
			sprintf(log,"playing-%s",(char*)pname);
			send_log(log);
			//播放标志位置位
			play_begin = 1;
		}

		if((xEventGroupValue & EVENTS_PLAY_AND_RECORD_BIT)!=0 || (xEventGroupValue & EVENTS_RECORD_BIT)!=0)
		{//录音部分初始化
			//获取录音文件名
		    recoder_new_pathname(rname);
		    app_trace_log("recorder: %s\n",rname);
			//初始化录音文件头
		    recoder_wav_init(wavheadrx,wavctrl.bps,wavctrl.samplerate);
			//打开录音文件
			strcpy(path,RECORD_PATH);
			strcat(path,(char*)rname);
			app_trace_log(path);
			app_trace_log("\n");
		    res=f_open(audiodev.file2,(const TCHAR*)path,FA_CREATE_ALWAYS|FA_WRITE);
			if (res != FR_OK)
			{
				app_trace_log("error:%x ,%s,%d\n",res,__FUNCTION__,__LINE__);
				continue;
			}
			//写入文件头
		    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);
		    if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error1;
			}
			f_sync(audiodev.file2);
			//将录音信息发送给log线程
			sprintf(log,"recording-%s",(char*)rname);
			send_log(log);
			//录音标志位置位
			record_begin = 1;
		}

		if ((xEventGroupValue & EVENTS_PLAY_BIT)!=0 || (xEventGroupValue & EVENTS_PLAY_AND_RECORD_BIT)!=0 ||
			(xEventGroupValue & EVENTS_RECORD_BIT)!=0)
		{//初始化公共部分
			if(wavctrl.bps==16)
			{
				WM8978_I2S_Cfg(2,0);	//飞利浦标准,16位数据长度
			}
			else if(wavctrl.bps==24)
			{
				WM8978_I2S_Cfg(2,2);	//飞利浦标准,24位数据长度
			}
			//初始化IIS时钟
			IIS_Init(wavctrl.bps,wavctrl.samplerate);
			if (play_begin == 1)
			{
				fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE,wavctrl.bps);
			}
			else if (record_begin == 1)
			{
				memset(audiodev.i2sbuf1,0x00,WAV_I2S_TX_DMA_BUFSIZE);
			}
			fillnum = WAV_I2S_TX_DMA_BUFSIZE/2;
			//启动dma开始播放
			HAL_I2SEx_TransmitReceive_DMA_A(&hi2s2,(uint16_t *)audiodev.i2sbuf1,(uint16_t *)audiodev.i2sbuf2,\
				WAV_I2S_TX_DMA_BUFSIZE/2);
			//开始录音播放
			//进入播放模式
			key_work_status = 1;
			stop_play_record = 0;
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		}

		if ((xEventGroupValue & EVENTS_FUN_STOP_BIT) != 0)
		{//停止播放
			stop_play_record = 1;
			app_trace_log("play stop\n");
			goto end;
		}

		//开始录音播放
		if ((play_begin == 1) || (record_begin == 1))
		{	//等待中断
			ulEventsToProcess = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
			if (ulEventsToProcess != 0)
			{
				if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//播放结束
				{
					app_trace_log("play end\n");
					stop_play_record = 1;
					goto end;
				}
				if (wavrxtxflag)
				{
					if(play_begin == 1)
					{
						fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,\
	                						wavctrl.bps);//填充buf1
					}
					if(record_begin == 1)
					{
						xSemaphoreTake(xSdioMutex,portMAX_DELAY);
	                	res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
		                if(res != FR_OK)
		                {
		                    app_trace_log("write error:%d %d\r\n",res,__LINE__);
		                }
		                else
		                {
		                    wavsize+=WAV_I2S_RX_DMA_BUFSIZE/2;
							f_sync(audiodev.file2);
						}
						xSemaphoreGive(xSdioMutex);
					}
				}
				else
				{
					if(play_begin == 1)
					{
						fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//填充buf1
					}
					if(record_begin == 1)
					{
						xSemaphoreTake(xSdioMutex,portMAX_DELAY);
						res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
		                if(res != FR_OK)
		                {
		                    app_trace_log("write error:%d %d\r\n",res,__LINE__);
		                }
		                else
		                {
		                    wavsize+=WAV_I2S_RX_DMA_BUFSIZE/2;
							f_sync(audiodev.file2);
						}
						xSemaphoreGive(xSdioMutex);
						if ((wavsize % 409600)==0)
							app_trace_log("writed :%d %d\n",res,__LINE__);
					}
				}
			}
end:
			if (stop_play_record == 1)
			{
				HAL_I2S_DMAStop(&hi2s2); //关闭dma
error1:
				if (record_begin == 1)
				{
				    wavheadrx->riff.ChunkSize=wavsize+36;		//整个文件的大小-8;
				    wavheadrx->data.ChunkSize=wavsize;		    //数据大小
				    res= f_lseek(audiodev.file2,0);			    //偏移到文件头.
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				    }
				    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);//写入头数据
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				    }
				    res= f_close(audiodev.file2);
					if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				    }
					//录音完成发送log
					sprintf(log,"complete-recording");
					send_log(log);
				}
				if (play_begin == 1)
				{
				    res= f_close(audiodev.file1);
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				    }
					//播放完成发送log
					sprintf(log,"complete-playing");
					send_log(log);
				}
			    app_trace_log("%s,%d,end\n",__FUNCTION__,__LINE__);
    			wavsize=0;
				play_begin = 0; //关闭播放
				record_begin = 0; //关闭录音
				key_work_status = 0;//进入待机模式
				HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
			}
		}
		else
		{
			//未播放时休眠500毫秒
			osDelay(500);
		}
	}
//    vPortFree(audiodev.i2sbuf1);
//    vPortFree(audiodev.i2sbuf2);
//	vPortFree(audiodev.tbuf);
//    vPortFree(audiodev.file1);
//    vPortFree(audiodev.file2);
//    vPortFree(wavheadrx);
//    vPortFree(pname);
//    vPortFree(rname);
//	osThreadTerminate(osThreadGetId());
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
	wavrxtxflag = 0;
	xHigherPriorityTaskWorken = pdFALSE;

	vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

	portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
	wavrxtxflag = 1;
	xHigherPriorityTaskWorken = pdFALSE;

	vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

	portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
}

