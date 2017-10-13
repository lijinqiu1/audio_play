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
//���ֲ��ſ�����
__audiodev audiodev;

//*********************************��������*****************************
uint32_t wavsize;  //wav���ݴ�С

uint8_t wavrxtxflag=0; //txrx����״̬��ʶ

//**********************************************************************

//�����ʼ��㹫ʽ:Fs=I2SxCLK/[256*(2*I2SDIV+ODD)]
//I2SxCLK=(HSE/pllm)*PLLI2SN/PLLI2SR
//һ��HSE=8Mhz
//pllm:��Sys_Clock_Set���õ�ʱ��ȷ����һ����8
//PLLI2SN:һ����192~432
//PLLI2SR:2~7
//I2SDIV:2~255
//ODD:0/1
//I2S��Ƶϵ����@pllm=8,HSE=8Mhz,��vco����Ƶ��Ϊ1Mhz
//����ʽ:������/10,PLLI2SN,PLLI2SR,I2SDIV,ODD
const uint16_t I2S_PSC_TBL[][5]=
{
	{800 ,256,5,12,1},		//8Khz������
	{1102,429,4,19,0},		//11.025Khz������
	{1600,213,2,13,0},		//16Khz������
	{2205,429,4, 9,1},		//22.05Khz������
	{3200,213,2, 6,1},		//32Khz������
	{4410,271,2, 6,0},		//44.1Khz������
	{4800,258,3, 3,1},		//48Khz������
	{8820,316,2, 3,1},		//88.2Khz������
	{9600,344,2, 3,1},  	//96Khz������
	{17640,361,2,2,0},  	//176.4Khz������
	{19200,393,2,2,0},  	//192Khz������
};
//1ʹ������ģ��
static void BT_Power(uint8_t enable)
{
	if (enable)
	{
		HAL_GPIO_WritePin(RESET_BT_PB11_GPIO_Port,RESET_BT_PB11_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MODE_BT_PB10_GPIO_Port,MODE_BT_PB10_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(REF_EN_PC4_GPIO_Port,REF_EN_PC4_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(REF_EN_PC4_GPIO_Port,REF_EN_PC4_Pin,GPIO_PIN_RESET);
	}
}
static void send_log(char *log)
{
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
	BaseType_t xStatus;
	char queuebuffer[QUEUE_LOG_ITEM_SIZE];

	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);
	sprintf(queuebuffer,"%04d/%02d/%02d-%02d/%02d/%02d/%03d    %s\r\n",\
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
	if(!ftemp || !buf) //�ڴ�����ɹ�
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
	res = f_read(ftemp,buf,512,&br); //��ȡ512�ֽ�����
    if (res != FR_OK)
    {
        app_trace_log("error %d %d %s\n",res,__LINE__,__FUNCTION__);
    	goto error1;
    }
    riff=(ChunkRIFF*)buf;  //��ȡRIFF��
    if(riff->Format == 0x45564157)//��WAV�ļ�"WAVE"
    {
        fmt=(ChunkFMT *)(buf + 12);
        fact=(ChunkFACT *)(buf + 12 + 8 + fmt->ChunkSize);//��ȡFACT��
        if(fact->ChunkID==0x74636166||fact->ChunkID==0x5453494C)
            wavx->datastart=12+8+fmt->ChunkSize+8+fact->ChunkSize;
        else
            wavx->datastart=12+8+fmt->ChunkSize;
        data=(ChunkDATA*)(buf+wavx->datastart); //��ȡDATA��
        if(data->ChunkID==0x61746164)//�����ɹ�
        {
            wavx->audioformat=fmt->AudioFormat;		//��Ƶ��ʽ
            wavx->nchannels=fmt->NumOfChannels;		//ͨ����
            wavx->samplerate=fmt->SampleRate;		//������
            wavx->bitrate=fmt->ByteRate*8;			//�õ�λ��
            wavx->blockalign=fmt->BlockAlign;		//�����
            wavx->bps=fmt->BitsPerSample;			//λ��,16/24/32λ

            wavx->datasize=data->ChunkSize;			//���ݿ��С
            wavx->datastart=wavx->datastart+8;		//��������ʼ�ĵط�.

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

//���buf
//buf:������
//tbuf:��ʱ������
//size:���������
//bits:λ��(16/24)
//����ֵ:���������ݸ���
static uint32_t wav_buffill(uint8_t *buf,uint8_t *tbuf,FIL*file,uint16_t size,uint8_t bits)
{
	uint16_t readlen=0;
	uint32_t bread = 0;
	uint16_t i = 0;
	uint8_t *p;
    FRESULT res;
	if(bits==24)//24bit��Ƶ,��Ҫ����һ��
	{
		readlen=(size/4)*3;							//�˴�Ҫ��ȡ���ֽ���
		res = f_read(file,tbuf,readlen,(UINT*)&bread);	//��ȡ����
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
		bread=(bread*4)/3;		//����Ĵ�С.
	}
	else
	{
		res = f_read(file,buf,size,(UINT*)&bread);//16bit��Ƶ,ֱ�Ӷ�ȡ����
        if (res == FR_OK)
        {
            if(bread<size)//����������,����0
            {
                for(i=bread;i<size-bread;i++)buf[i]=0;
            }
        }
	}
	return bread;
}

//�õ���ǰ����ʱ��
//fx:�ļ�ָ��
//wavx:wav���ſ�����
//static void wav_get_curtime(FIL*fx,__wavctrl *wavx)
//{
//	long long fpos;
// 	wavx->totsec=wavx->datasize/(wavx->bitrate/8);	//�����ܳ���(��λ:��)
//	fpos=fx->fptr-wavx->datastart; 					//�õ���ǰ�ļ����ŵ��ĵط�
//	wavx->cursec=fpos*wavx->totsec/wavx->datasize;	//��ǰ���ŵ��ڶ�������?
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
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK)
    {
        Error_Handler();
    }
}

//ͨ��ʱ���ȡ�ļ���
//������SD������,��֧��FLASH DISK����
//��ϳ�:����"0:RECORDER/REC20120321210633.wav"���ļ���
static void recoder_new_pathname(uint8_t *pname)
{
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);

	sprintf((char*)pname,"%04d-%02d-%02d-%02d-%02d-%02d.wav",\
		 dat.Year+2000,dat.Month,dat.Date,tim.Hours,tim.Minutes,tim.Seconds);
}

//��ʼ��WAVͷ.
void recoder_wav_init(__WaveHeader* wavhead,uint32_t DataFormat,uint32_t AudioFreq) //��ʼ��WAVͷ
{
	wavhead->riff.ChunkID=0X46464952;	//"RIFF"
	wavhead->riff.ChunkSize=0;			//��δȷ��,�����Ҫ����
	wavhead->riff.Format=0X45564157; 	//"WAVE"
	wavhead->fmt.ChunkID=0X20746D66; 	//"fmt "
	wavhead->fmt.ChunkSize=16; 			//��СΪ16���ֽ�
	wavhead->fmt.AudioFormat=0X01; 		//0X01,��ʾPCM;0X01,��ʾIMA ADPCM
 	wavhead->fmt.NumOfChannels=2;		//˫����
 	wavhead->fmt.SampleRate=AudioFreq;		//16Khz������ ��������
 	wavhead->fmt.ByteRate=wavhead->fmt.SampleRate*4;//�ֽ�����=������*ͨ����*(ADCλ��/8)
 	wavhead->fmt.BlockAlign=4;			//���С=ͨ����*(ADCλ��/8)
 	wavhead->fmt.BitsPerSample=DataFormat;		//16λPCM
   	wavhead->data.ChunkID=0X61746164;	//"data"
 	wavhead->data.ChunkSize=0;			//���ݴ�С,����Ҫ����
}

//*******************************************************************************
//��ȡ�����б� ���������ļ�����
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
//��Ƶ�����߳�
//*******************************************************************************
void AudioController_Task(void const * argument)
{
	EventBits_t xEventGroupValue;
	char log[40];
	static uint8_t ble_status = 0;//����״̬����ʼ״̬�����ر�
	const EventBits_t xBitsToWaitFor = (EVENTS_VOL_UP_BIT|
		                                  EVENTS_VOL_DOWN_BIT|
		                                  EVENTS_FUN_BLE_CHANGE_BIT|
		                                  EVENTS_FUN_BLE_PAIR_BIT);
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
		//����ģʽ
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
			//��¼log
			sprintf(log,"volume+");
			send_log(log);
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
			//��¼log
			sprintf(log,"volume-");
			send_log(log);
		}
		if((xEventGroupValue&EVENTS_FUN_BLE_CHANGE_BIT)!=0)
		{//ʹ������/�����л�
		    if(ble_status)
		    {
				ble_status = 0;
				BT_Power(0);
			}
			else
			{
				ble_status = 1;
				BT_Power(1);
			}
		}
		if((xEventGroupValue&EVENTS_FUN_BLE_PAIR_BIT)!=0)
		{//�������
			sprintf(log,"BLE Pairing");
			send_log(log);
			HAL_GPIO_WritePin(PAIR_BT_PB1_GPIO_Port,PAIR_BT_PB1_Pin,GPIO_PIN_RESET);
			osDelay(100);
			HAL_GPIO_WritePin(PAIR_BT_PB1_GPIO_Port,PAIR_BT_PB1_Pin,GPIO_PIN_SET);
		}
	}

}
//*******************************************************************************
//��Ƶ����¼���߳�
//*******************************************************************************
void AudioPlay_Task(void const * argument)
{
	//��ȡmusic�ļ�������Ƶ�ļ�
	FRESULT res;
	EventBits_t xEventGroupValue;
    FATFS fs;
	DIR recdir;
	//wav�ļ�ͷ
	__WaveHeader *wavheadrx;
	__wavctrl wavctrl;		//WAV���ƽṹ��
	uint8_t file_count;
	//¼���ļ���
	uint8_t *rname;
	//�����ļ���
	uint8_t *pname;
	uint32_t bw;
	uint32_t fillnum;
	uint32_t ulEventsToProcess;
	//��ʼ����
	uint8_t play_begin = 0;
	//��ʼ¼��
	uint8_t record_begin = 0;
	//ֹͣ��Ƶ����
	uint8_t stop_play_record = 0;
	//���·��
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
	//����SD��
	res = f_mount(&fs,(const TCHAR*)SD_Path,0);
	if (res !=FR_OK)
	{
	 app_trace_log("err %s,%d\n",__FUNCTION__,__LINE__);
	 while(1);
	}
	//��ȡ���������ļ�
	file_count = Get_Play_List();
	//��¼���ļ��У����û�д���
	while(f_opendir(&recdir,"0:/RECORD"))
	{
		res = f_mkdir("0:/RECORD");
		if (res != FR_OK)
		{
			app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			goto error2 ;
		}
	}
	f_closedir(&recdir);
    WM8978_Init();
	//���벥�Ż���
	audiodev.file1=(FIL*)pvPortMalloc(sizeof(FIL));
	//����¼������
	audiodev.file2=(FIL*)pvPortMalloc(sizeof(FIL));
	//���벥���ļ�handle
	audiodev.i2sbuf1=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);
	//����¼���ļ�handle
	audiodev.i2sbuf2=pvPortMalloc(WAV_I2S_RX_DMA_BUFSIZE);
	//�����������ʱʹ��
	audiodev.tbuf=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);
	//����¼���ļ�ͷ
	wavheadrx = (__WaveHeader*)pvPortMalloc(sizeof(__WaveHeader));
	//����¼���ļ���
	rname = pvPortMalloc(40);
	//���벥���ļ���
	pname = pvPortMalloc(40);
	if(!audiodev.file1 || !audiodev.file2 || !audiodev.i2sbuf1 || !audiodev.i2sbuf2 || !audiodev.tbuf ||\
			!wavheadrx || !wavheadrx || !rname || !pname)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	//Ĭ��¼�������ʡ�����λ��
	wavctrl.bps = 16;
	wavctrl.samplerate = I2S_AUDIOFREQ_16K;
	for(;;)
	{	//�Ƕ���ģʽ
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
		//�ȴ��¼�ͬ��
//		xEventGroupSync(xEventGroup,
//						xBitsToWaitFor,
//						uxAllSyncBits,
//						portMAX_DELAY);


		if((xEventGroupValue & EVENTS_PLAY_BIT)!=0 || (xEventGroupValue & EVENTS_PLAY_AND_RECORD_BIT)!=0)
		{//���Ų��ֳ�ʼ��
			//��ȡ��������ļ���
			Get_Play_Song(pname,file_count);
			//�õ��ļ�����Ϣ
			res = wav_decode_init(pname,&wavctrl);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error2;
			}
			strcpy(path,MUSIC_PATH);
			strcat(path,(const char*)pname);
			//�򿪲����ļ�
			res=f_open(audiodev.file1,(const TCHAR*)path,FA_READ);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				continue;
			}
			//���������ļ�ͷ
			res = f_lseek(audiodev.file1, wavctrl.datastart);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error1;
			}
			//��������Ϣ���͸�log�߳�
			sprintf(log,"playing-%s",(char*)pname);
			send_log(log);
			//���ű�־λ��λ
			play_begin = 1;
		}

		if((xEventGroupValue & EVENTS_PLAY_AND_RECORD_BIT)!=0 || (xEventGroupValue & EVENTS_RECORD_BIT)!=0)
		{//¼�����ֳ�ʼ��
			//��ȡ¼���ļ���
		    recoder_new_pathname(rname);
		    app_trace_log("recorder: %s\n",rname);
			//��ʼ��¼���ļ�ͷ
		    recoder_wav_init(wavheadrx,wavctrl.bps,wavctrl.samplerate);
			//��¼���ļ�
			strcpy(path,RECORD_PATH);
			strcat(path,(char*)rname);
			app_trace_log(path);
			app_trace_log("\n");
		    res=f_open(audiodev.file2,(const TCHAR*)path,FA_CREATE_ALWAYS|FA_WRITE);
			if (res != FR_OK)
			{
				app_trace_log("error:%d ,%s,%d\n",res,__FUNCTION__,__LINE__);
				continue;
			}
			//д���ļ�ͷ
		    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);
		    if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error1;
			}
			f_sync(audiodev.file2);
			//��¼����Ϣ���͸�log�߳�
			sprintf(log,"recording-%s",(char*)rname);
			send_log(log);
			//¼����־λ��λ
			record_begin = 1;
		}

		if ((xEventGroupValue & EVENTS_PLAY_BIT)!=0 || (xEventGroupValue & EVENTS_PLAY_AND_RECORD_BIT)!=0 ||
			(xEventGroupValue & EVENTS_RECORD_BIT)!=0)
		{//��ʼ����������
			if(wavctrl.bps==16)
			{
				WM8978_I2S_Cfg(2,0);	//�����ֱ�׼,16λ���ݳ���
			}
			else if(wavctrl.bps==24)
			{
				WM8978_I2S_Cfg(2,2);	//�����ֱ�׼,24λ���ݳ���
			}
			//��ʼ��IISʱ��
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
			//����dma��ʼ����
			HAL_I2SEx_TransmitReceive_DMA_A(&hi2s2,(uint16_t *)audiodev.i2sbuf1,(uint16_t *)audiodev.i2sbuf2,\
				WAV_I2S_TX_DMA_BUFSIZE/2);
			//��ʼ¼������
			//���벥��ģʽ
			key_work_status = 1;
			stop_play_record = 0;
		}

		if ((xEventGroupValue & EVENTS_FUN_STOP_BIT) != 0)
		{//ֹͣ����
			stop_play_record = 1;
			goto end;
		}

		//��ʼ¼������
		if ((play_begin == 1) || (record_begin == 1))
		{	//�ȴ��ж�
			ulEventsToProcess = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
			if (ulEventsToProcess != 0)
			{
				if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//���Ž���
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
	                						wavctrl.bps);//���buf1
					}
					if(record_begin == 1)
					{
	                	res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
		                if(res != FR_OK)
		                {
		                    app_trace_log("write error:%d %d\r\n",res,__LINE__);
		                }
		                else
		                {
		                    wavsize+=WAV_I2S_RX_DMA_BUFSIZE/2;
						}
					}
				}
				else
				{
					if(play_begin == 1)
					{
						fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//���buf1
					}
					if(record_begin == 1)
					{
						res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
		                if(res != FR_OK)
		                {
		                    app_trace_log("write error:%d %d\r\n",res,__LINE__);
		                }
		                else
		                {
		                    wavsize+=WAV_I2S_RX_DMA_BUFSIZE/2;
						}
					}
				}
			}
end:
			if (stop_play_record == 1)
			{
				HAL_I2S_DMAStop(&hi2s2); //�ر�dma
error1:
				if (record_begin == 1)
				{
				    wavheadrx->riff.ChunkSize=wavsize+36;		//�����ļ��Ĵ�С-8;
				    wavheadrx->data.ChunkSize=wavsize;		    //���ݴ�С
				    res= f_lseek(audiodev.file2,0);			    //ƫ�Ƶ��ļ�ͷ.
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				    }
				    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);//д��ͷ����
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				    }
				    res= f_close(audiodev.file2);
					if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				    }
					//¼����ɷ���log
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
					//������ɷ���log
					sprintf(log,"complete-playing");
					send_log(log);
				}
			    app_trace_log("%s,%d,end\n",__FUNCTION__,__LINE__);
    			wavsize=0;
				play_begin = 0; //�رղ���
				record_begin = 0; //�ر�¼��
				key_work_status = 0;//�������ģʽ
			}
		}
		else
		{
			//δ����ʱ����500����
			osDelay(500);
		}
	}
error2:
    vPortFree(audiodev.i2sbuf1);
    vPortFree(audiodev.i2sbuf2);
	vPortFree(audiodev.tbuf);
    vPortFree(audiodev.file1);
    vPortFree(audiodev.file2);
    vPortFree(wavheadrx);
    vPortFree(pname);
    vPortFree(rname);
	for(;;)
	{
		osDelay(1000);
	}
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
