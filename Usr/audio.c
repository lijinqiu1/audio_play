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

#if defined(PLAY_WITH_LIST)
osThreadId audioplaywithlistTaskHandle;
#elif defined(PLAY_WITH_RNG)
osThreadId audioplayTaskHandle;
#endif
osThreadId audiocontrollerHandle;
extern EventGroupHandle_t xEventGroup;
extern QueueHandle_t xQueueLog;
extern SemaphoreHandle_t xSdioMutex;
//���ֲ��ſ�����
__audiodev audiodev;

//*********************************��������*****************************
uint32_t wavsize;  //wav���ݴ�С
#if defined (IIS_DMA_A)
uint8_t wavrxtxflag=0; //txrx����״̬��ʶ
#else
uint8_t wavtxintflag = 0;
uint8_t wavrxintflag = 0;
uint8_t wavtxflag=0; //tx����״̬��ʶ
uint8_t wavrxflag=0; //rx����״̬��ʶ
#endif
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
//���ʽ:������/10,PLLI2SN,PLLI2SR,I2SDIV,ODD
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
#if defined(PLAY_WITH_LIST)
//��ǰ���ŵ��ļ�����
static uint8_t cur_file_index = 0;
static uint8_t *play_list;
#endif

//1ʹ������ģ��
static void BT_Power(uint8_t enable)
{
#if defined(F429_ZET6)
	if (enable)
	{
		HAL_GPIO_WritePin(PAIR_BT_GPIO_Port,PAIR_BT_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MODE_BT_GPIO_Port,MODE_BT_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(REF_EN_GPIO_Port,REF_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(RESET_BT_GPIO_Port,RESET_BT_Pin,GPIO_PIN_RESET);
        osDelay(10);
		HAL_GPIO_WritePin(RESET_BT_GPIO_Port,RESET_BT_Pin,GPIO_PIN_SET);
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

static void save_task_log(FIL *file,char *log)
{
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
	char queuebuffer[QUEUE_LOG_ITEM_SIZE];

	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);
	sprintf(queuebuffer,"%04d/%02d/%02d-%02d/%02d/%02d/%03d    %s\n",\
		dat.Year+2000,dat.Month,dat.Date,\
		tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds,\
		log);
//	xSemaphoreTake(xSdioMutex,portMAX_DELAY);
	f_puts((const TCHAR*)queuebuffer,file);
//	xSemaphoreGive(xSdioMutex);
}

static FRESULT wav_decode_init(uint8_t *fname, __wavctrl* wavx)
{
	FIL *ftemp;
	FRESULT res = FR_OK;
	uint8_t *buf;
	uint32_t br = 0;

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

	res = f_open(ftemp,(const TCHAR*)fname,FA_READ);
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
/*
            app_trace_log("wavx->audioformat:%d\r\n",wavx->audioformat);
            app_trace_log("wavx->nchannels:%d\r\n",wavx->nchannels);
            app_trace_log("wavx->samplerate:%d\r\n",wavx->samplerate);
            app_trace_log("wavx->bitrate:%d\r\n",wavx->bitrate);
            app_trace_log("wavx->blockalign:%d\r\n",wavx->blockalign);
            app_trace_log("wavx->bps:%d\r\n",wavx->bps);
            app_trace_log("wavx->datasize:%d\r\n",wavx->datasize);
            app_trace_log("wavx->datastart:%d\r\n",wavx->datastart);
            */
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

//	xSemaphoreTake(xSdioMutex,portMAX_DELAY);
	if(bits!=24)//24bit��Ƶ,��Ҫ����һ��
	{
		res = f_read(file,buf,size,(UINT*)&bread);//16bit��Ƶ,ֱ�Ӷ�ȡ����
        if (res == FR_OK)
        {
            if(bread<size)//����������,����0
            {
                for(i=bread;i<size-bread;i++)buf[i]=0;
            }
        }
		else
		{
            app_trace_log("%s,%d,err:%d\n",__FUNCTION__,__LINE__,res);
            //app_trace_log("fs 0x%x\n",file->fs);
			APP_ERROR_CHECK(res);
		}
	}
	else
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
//	xSemaphoreGive(xSdioMutex);
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

static void task_log_new_pathname(uint8_t *pname)
{
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);

	sprintf((char*)pname,"%04d-%02d-%02d-%02d-%02d-%02d.txt",\
		 dat.Year+2000,dat.Month,dat.Date,tim.Hours,tim.Minutes,tim.Seconds);
    app_trace_log("%s\n",pname);
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
static uint8_t Get_Song_Count(void)
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
#if defined(PLAY_WITH_RNG)
static void Get_Play_Song(uint8_t *fname,uint8_t file_count)
{
	uint32_t number;

	number = HAL_RNG_GetRandomNumber(&hrng);
	app_trace_log("%s,num %uld, count %d\n",__FUNCTION__,number,file_count);
	number = number % file_count + 1;
	sprintf((char*)fname,"%03d.wav",number);
}
#endif

static void Get_Play_Rng_List(uint8_t *list, uint8_t file_count)
{
	uint8_t i,j;
	uint32_t number;
	number = HAL_RNG_GetRandomNumber(&hrng);
	list[0] = number % file_count;
	for (i = 1; i < file_count; i++)
	{
		number = HAL_RNG_GetRandomNumber(&hrng);
		list[i] = number % file_count;
		for(j = 0; j < i; j++)
		{
			if (list[i] == list[j])
			{
				i--;
			}
		}
	}
	for (i = 0; i < file_count; i++)
	{
		list[i]++;
	}
}
//*******************************************************************************
//��Ƶ�����߳�
//*******************************************************************************
#if defined(PLAY_WITH_LIST)
void AudioController_Task(void const * argument)
{
	EventBits_t xEventGroupValue;
	//ѵ��������־
	FIL *log_fil;
	FRESULT res;
	char log_fil_name[40];
	char log_path[40];
	char log[40];
	static uint8_t usb_status = 0;
	static uint8_t ble_status = 0;//����״̬����ʼ״̬�����ر�
    app_trace_log("AudioController_Task begin\n");
	log_fil = (FIL*)pvPortMalloc(sizeof(FIL));
    if (!log_fil)
    {
        app_trace_log("log_fil Malloc faild %s,%d\n",__FUNCTION__,__LINE__);
        while(1);
    }
	const EventBits_t xBitsToWaitFor = (EVENTS_VOL_UP_BIT|
		                                  EVENTS_VOL_DOWN_BIT|
		                                  EVENTS_FUN_BLE_CHANGE_BIT|
		                                  EVENTS_BLE_PAIR_BIT|
		                                  EVENTS_FUN_USB_BIT|
										  EVENTS_ASK_BIT|
										  EVENTS_PLAY_AND_RECORD_END_BIT|
										  EVENTS_PLAY_NEW_SONG_BIT|
										  EVENTS_TASK_LOG_CREATE_BIT|
										  EVENTS_PLAY_CASE_BIT);
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
		{//��������
			volume+=5;
			if (volume > WM8978_DAC_MAX_VOL)
			{
				volume = WM8978_DAC_MAX_VOL;
			}
			//WM8978_HPvol_Set(volume,volume);
			//WM8978_SPKvol_Set(volume);
            WM8978_DACvol_Set(volume);
			//��¼log
			sprintf(log,"volume+");
			save_task_log(log_fil,log);
			send_log(log);
			app_trace_log("volume+\n");
		}
		if((xEventGroupValue&EVENTS_VOL_DOWN_BIT)!=0)
		{//������С
			volume -= 5;
			if(volume < 0)
			{
				volume = 0;
			}
			//WM8978_HPvol_Set(volume,volume);
			//WM8978_SPKvol_Set(volume);
            WM8978_DACvol_Set(volume);
			//��¼log
			sprintf(log,"volume-");
			save_task_log(log_fil,log);
			send_log(log);
			app_trace_log("volume-\n");
		}
		if((xEventGroupValue&EVENTS_FUN_BLE_CHANGE_BIT)!=0)
		{//ʹ������/�����л�
		    if(ble_status)
		    {
				ble_status = 0;
				BT_Power(0);
				sprintf(log,"BLE CLOSE");
				send_log(log);
				app_trace_log("BLE CLOSE!\n");
			}
			else
			{
				ble_status = 1;
				BT_Power(1);
				sprintf(log,"BLE OPEN");
				send_log(log);
				app_trace_log("BLE OPEN!\n");
			}
		}
        if((xEventGroupValue&EVENTS_BLE_PAIR_BIT)!=0)
        {
            if (ble_status == 1)
            {
                HAL_GPIO_WritePin(PAIR_BT_GPIO_Port, PAIR_BT_Pin, GPIO_PIN_RESET);
                osDelay(100);
                HAL_GPIO_WritePin(PAIR_BT_GPIO_Port, PAIR_BT_Pin, GPIO_PIN_SET);
    			sprintf(log,"BLE PAIR");
				send_log(log);
            }
        }
		if((xEventGroupValue&EVENTS_FUN_USB_BIT)!=0)
		{//�򿪹ر�usb
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
		{//����
			sprintf(log,"report");
			send_log(log);
			save_task_log(log_fil,log);
			app_trace_log("report\n");
		}
		if(xEventGroupValue&EVENTS_TASK_LOG_CREATE_BIT)
		{//���������¼�ļ�
			task_log_new_pathname((uint8_t *)log_fil_name);
			//��¼���ļ�
			strcpy(log_path,RECORD_PATH);
			strcat(log_path,(char*)log_fil_name);
			res=f_open(log_fil,(const TCHAR*)log_path,FA_CREATE_ALWAYS|FA_WRITE);
			if (res != FR_OK)
			{
				app_trace_log("error:%x ,%s,%d\n",res,__FUNCTION__,__LINE__);
			}
            //���沥���ļ���
			sprintf(log,"PLAY %03d.wav",play_list[0]);
			save_task_log(log_fil,log);
            app_trace_log("task log create\n");
		}
		if(xEventGroupValue&EVENTS_PLAY_AND_RECORD_END_BIT)
		{//�������
			sprintf(log,"Task Completed");
			save_task_log(log_fil,log);
            f_sync(log_fil);
			res=f_close(log_fil);
			if (res != FR_OK)
			{
				app_trace_log("error:%x ,%s,%d\n",res,__FUNCTION__,__LINE__);
			}
            app_trace_log("Task Completed\n");
		}
		if(xEventGroupValue&EVENTS_PLAY_CASE_BIT)
		{//����ȡ��
			sprintf(log,"Task Case");
			save_task_log(log_fil,log);
            f_sync(log_fil);
			res=f_close(log_fil);
			if (res != FR_OK)
			{
				app_trace_log("error:%x ,%s,%d\n",res,__FUNCTION__,__LINE__);
			}
            app_trace_log("Task Case\n");
		}
		if(xEventGroupValue & EVENTS_PLAY_NEW_SONG_BIT)
		{//������һ��
			sprintf(log,"PLAY %03d.wav",play_list[cur_file_index]);
			save_task_log(log_fil,log);
		}
	}

}
#elif defined(PLAY_WITH_RNG)
//*******************************************************************************
//��Ƶ�����߳�
//*******************************************************************************
void AudioController_Task(void const * argument)
{
	EventBits_t xEventGroupValue;
	char log[40];
	static uint8_t usb_status = 0;
	static uint8_t ble_status = 0;//����״̬����ʼ״̬�����ر�
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
			//��¼log
			sprintf(log,"volume-");
			send_log(log);
			app_trace_log("volume-\n");
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
				sprintf(log,"BLE Pairing");
				send_log(log);
			}
		}
		if((xEventGroupValue&EVENTS_FUN_USB_BIT)!=0)
		{//�򿪹ر�usb
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

#endif

#if defined(PLAY_WITH_LIST)
//*******************************************************************************
//���б�����Ƶ�ļ���ͬʱ¼��
//*******************************************************************************
void AudioPlay_With_List_Task(void const *argument)
{
	FRESULT res;
//    FATFS fs;
	EventBits_t xEventGroupValue;
	DIR recdir;
	//wav�ļ�ͷ
	__WaveHeader *wavheadrx;
	__wavctrl wavctrl;		//WAV���ƽṹ��
	//�ļ�����
	uint8_t file_count = 0;
	//¼���ļ���
	uint8_t *rname;
	//�����ļ���
	uint8_t *pname;
	uint32_t bw;
	uint32_t fillnum;
	uint32_t ulEventsToProcess;
	//��ʼ����
	uint8_t play_begin = 0;
	//ֹͣ��Ƶ����
	uint8_t stop_play_record = 0;
	//���·��
	uint8_t path[40];
	char log[40];
	const EventBits_t xBitsToWaitFor = (EVENTS_FUN_STOP_BIT |
	                                     EVENTS_PLAY_AND_RECORD_BIT);
    app_trace_log("%s begin\n",__FUNCTION__);
	
	//��¼���ļ��У����û�д���
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
    //��ȡ�����ļ���Ŀ
    file_count = Get_Song_Count();
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
	//���벥���б�
	play_list = pvPortMalloc((file_count/8 + 1)*8);
	if(!audiodev.file1 || !audiodev.file2 || !audiodev.i2sbuf1 || !audiodev.i2sbuf2 || !audiodev.tbuf ||\
			!wavheadrx || !wavheadrx || !rname || !pname || !play_list)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		NVIC_SystemReset();
	}
	//Ĭ��¼�������ʡ�����λ��
	wavctrl.bps = 16;
	wavctrl.samplerate = I2S_AUDIOFREQ_16K;
	for (;;)
	{
		//�Ƕ���ģʽ
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
		if (xEventGroupValue & EVENTS_PLAY_AND_RECORD_BIT)
		{
			/**********************************************************************
			�������Ų��ֳ�ʼ��
			***********************************************************************/
			//��ȡ�����ļ��б�
			Get_Play_Rng_List(play_list,file_count);
			//��ȡ��һ�������ļ�
			sprintf((char*)pname,"%03d.wav",play_list[cur_file_index]);
			//����ļ�·��
			strcpy((char *)path,MUSIC_PATH);
			strcat((char *)path,(const TCHAR*)pname);
			app_trace_log((const char *)path);
			app_trace_log("\n");
			//�õ��ļ�����Ϣ
			res = wav_decode_init(path,&wavctrl);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				stop_play_record = 1;
				goto end;
			}
			//�򿪲����ļ�
			res=f_open(audiodev.file1,(const TCHAR*)path,FA_READ);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto end;
			}
			//���������ļ�ͷ
			res = f_lseek(audiodev.file1, wavctrl.datastart);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error1;
			}
            
            app_trace_log("fs 0x%x\n",audiodev.file1->fs);
			//������Ƶ��������
			fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE,wavctrl.bps);
			fillnum = WAV_I2S_TX_DMA_BUFSIZE/2;
			//��������Ϣ���͸�log�߳�
			sprintf(log,"playing-%s",(char*)pname);
			send_log(log);
			/**********************************************************************
			¼�����ֳ�ʼ��
			***********************************************************************/
			//��ȡ¼���ļ���
		    recoder_new_pathname(rname);
		    app_trace_log("recorder: %s\n",rname);
			//��ʼ��¼���ļ�ͷ
		    recoder_wav_init(wavheadrx,wavctrl.bps,wavctrl.samplerate);
			//��¼���ļ�
			strcpy((char *)path,RECORD_PATH);
			strcat((char *)path,(char*)rname);
			app_trace_log((char *)path);
			app_trace_log("\n");
		    res=f_open(audiodev.file2,(const TCHAR*)path,FA_CREATE_ALWAYS|FA_WRITE);
			if (res != FR_OK)
			{
				app_trace_log("error:%x ,%s,%d\n",res,__FUNCTION__,__LINE__);
				goto end;
			}
//			xSemaphoreTake(xSdioMutex,portMAX_DELAY);
			//д���ļ�ͷ
		    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);
		    if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error1;
			}
//			xSemaphoreGive(xSdioMutex);
			f_sync(audiodev.file2);
			//��¼����Ϣ���͸�log�߳�
			sprintf(log,"recording-%s",(char*)rname);
			send_log(log);
			/**********************************************************************
			��ʼ����������
			***********************************************************************/
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
			//��ʼ¼������
			//���벥��ģʽ
			play_begin = 1;
			key_work_status = KEY_WORK_STATUS_PLAY;
			stop_play_record = 0;
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
            xEventGroupSetBits(xEventGroup, EVENTS_TASK_LOG_CREATE_BIT);
            #if defined (IIS_DMA_A)
			//����dma��ʼ����
			HAL_I2SEx_TransmitReceive_DMA_A(&hi2s2,(uint16_t *)audiodev.i2sbuf1,(uint16_t *)audiodev.i2sbuf2,\
				WAV_I2S_TX_DMA_BUFSIZE/2);
            #else
			//����dma��ʼ����
			HAL_I2SEx_TransmitReceive_DMA_B(&hi2s2,(uint16_t *)audiodev.i2sbuf1,(uint16_t *)audiodev.i2sbuf2,\
				WAV_I2S_TX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2);
            #endif
		}
		if ((xEventGroupValue & EVENTS_FUN_STOP_BIT) != 0)
		{//ֹͣ����
			stop_play_record = 1;
            xEventGroupSetBits(xEventGroup, EVENTS_PLAY_CASE_BIT);
			app_trace_log("play stop\n");
			goto end;
		}
		if (play_begin == 1)
		{
			//�ȴ��ж�
			ulEventsToProcess = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
			if (ulEventsToProcess != 0)
			{
				if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//��ǰ�ļ������꣬����һ��һ���ļ�
				{
					cur_file_index++;
					if(cur_file_index < file_count)
					{
						//�رյ�ǰ�����ļ�
						res= f_close(audiodev.file1);
						if (res != FR_OK)
						{
							app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
                            goto end;
						}
						//��ȡ��һ�������ļ�
						sprintf((char*)pname,"%03d.wav",play_list[cur_file_index]);
						//����ļ�·��
						strcpy((char *)path,MUSIC_PATH);
						strcat((char *)path,(const TCHAR*)pname);
						app_trace_log((char *)path);
						app_trace_log("\n");
						//�򿪲����ļ�
						res=f_open(audiodev.file1,(const TCHAR*)path,FA_READ);
						if (res != FR_OK)
						{
							app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
                            goto end;
						}
						f_sync(audiodev.file1);
						//���������ļ�ͷ
						res = f_lseek(audiodev.file1, wavctrl.datastart);
						if (res != FR_OK)
						{
							app_trace_log("error:%d %s,%d\n",res,__FUNCTION__,__LINE__);
							goto error1;
						}
						//������Ƶ��������
						fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE,wavctrl.bps);
						fillnum = WAV_I2S_TX_DMA_BUFSIZE/2;
						//֪ͨ�洢����log�ļ�
						xEventGroupSetBits(xEventGroup,EVENTS_PLAY_NEW_SONG_BIT);
					}
					else
					{
						app_trace_log("play end\n");
						xEventGroupSetBits(xEventGroup,EVENTS_PLAY_AND_RECORD_END_BIT);
						stop_play_record = 1;
						goto end;
					}
				}
#if defined (IIS_DMA_A)
				if (wavrxtxflag)
				{
					fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,\
										wavctrl.bps);//���buf1
//					xSemaphoreTake(xSdioMutex,portMAX_DELAY);
					res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
					if(res == FR_OK)
					{
						wavsize+=WAV_I2S_RX_DMA_BUFSIZE/2;
						//f_sync(audiodev.file2);
					}
					else
					{
						app_trace_log("write error:%d %d\r\n",res,__LINE__);
					}
//					xSemaphoreGive(xSdioMutex);
				}
				else
				{
					fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//���buf1
//					xSemaphoreTake(xSdioMutex,portMAX_DELAY);
					res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
					if(res == FR_OK)
					{
						wavsize+=WAV_I2S_RX_DMA_BUFSIZE/2;
						//f_sync(audiodev.file2);
					}
					else
					{
						app_trace_log("write error:%d %d\r\n",res,__LINE__);
					}
//					xSemaphoreGive(xSdioMutex);
				}
#else
                if(wavtxintflag)
                {
                    wavtxintflag = 0;
                    if(wavtxflag)
                    {
                        fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,\
                                                                wavctrl.bps);//���buf1
                    }
                    else
                    {
                        fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//���buf1
                    }
                }
                if(wavrxintflag)
                {
                    wavrxintflag = 0;
                    if(wavrxflag)
                    {
                        res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
    					if(res == FR_OK)
    					{
    						wavsize+=WAV_I2S_RX_DMA_BUFSIZE/2;
    						//f_sync(audiodev.file2);
    					}
    					else
    					{
    						app_trace_log("write error:%d %d\r\n",res,__LINE__);
    					}
                    }
                    else
                    {
                        res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
    					if(res == FR_OK)
    					{
    						wavsize+=WAV_I2S_RX_DMA_BUFSIZE/2;
    						//f_sync(audiodev.file2);
    					}
    					else
    					{
    						app_trace_log("write error:%d %d\r\n",res,__LINE__);
    					}
                    }
                }
#endif
			}

end:
			if (stop_play_record == 1)
			{
				HAL_I2S_DMAStop(&hi2s2); //�ر�dma
error1:
				//����¼���ļ�
			    wavheadrx->riff.ChunkSize=wavsize+36;		//�����ļ��Ĵ�С-8;
			    wavheadrx->data.ChunkSize=wavsize;		    //���ݴ�С
			    res= f_lseek(audiodev.file2,0);			    //ƫ�Ƶ��ļ�ͷ.
			    if (res != FR_OK)
			    {
			        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			    }
				xSemaphoreTake(xSdioMutex,portMAX_DELAY);
			    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);//д��ͷ����
			    if (res != FR_OK)
			    {
			        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			    }
				xSemaphoreGive(xSdioMutex);
				//�ر�¼���ļ�
			    res= f_close(audiodev.file2);
				if (res != FR_OK)
			    {
			        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			    }
				//¼����ɷ���log
				sprintf(log,"complete-recording");
				send_log(log);
				//�رղ����ļ�
			    res= f_close(audiodev.file1);
			    if (res != FR_OK)
			    {
			        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			    }
				//������ɷ���log
				sprintf(log,"complete-playing");
				send_log(log);
			    app_trace_log("%s,%d,end\n",__FUNCTION__,__LINE__);
    			wavsize=0;
				play_begin = 0; //�رղ���
				cur_file_index = 0;//��ǰ������������
				key_work_status = KEY_WORK_STATUS_READY;//�������ģʽ
				HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
			}
		}
		else
		{
			//δ����ʱ����500����
			osDelay(500);
		}
	}
}
#elif defined(PLAY_WITH_RNG)
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
	uint8_t file_count ;
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
	APP_ERROR_CHECK(res);
	//��ȡ���������ļ�
	file_count = Get_Song_Count();
	//��¼���ļ��У����û�д���
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
		NVIC_SystemReset();
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
			//����ļ�·��
			strcpy(path,MUSIC_PATH);
			strcat(path,(const TCHAR*)fname);
			app_trace_log(path);
			app_trace_log("\n");
			//�õ��ļ�����Ϣ
			res = wav_decode_init(path,&wavctrl);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				stop_play_record = 1;
				goto end;
			}
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
				app_trace_log("error:%x ,%s,%d\n",res,__FUNCTION__,__LINE__);
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
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		}

		if ((xEventGroupValue & EVENTS_FUN_STOP_BIT) != 0)
		{//ֹͣ����
			stop_play_record = 1;
			app_trace_log("play stop\n");
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
						xSemaphoreTake(xSdioMutex,portMAX_DELAY);
	                	res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
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
						fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//���buf1
					}
					if(record_begin == 1)
					{
						xSemaphoreTake(xSdioMutex,portMAX_DELAY);
						res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
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
				HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
			}
		}
		else
		{
			//δ����ʱ����500����
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
#endif

#if defined (IIS_DMA_A)

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
	wavrxtxflag = 0;
	xHigherPriorityTaskWorken = pdFALSE;
#if defined(PLAY_WITH_LIST)
	vTaskNotifyGiveFromISR(audioplaywithlistTaskHandle,&xHigherPriorityTaskWorken);
#elif defined(PLAY_WITH_RNG)
	vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);
#endif

	portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
	wavrxtxflag = 1;
	xHigherPriorityTaskWorken = pdFALSE;
#if defined(PLAY_WITH_LIST)
	vTaskNotifyGiveFromISR(audioplaywithlistTaskHandle,&xHigherPriorityTaskWorken);
#elif defined(PLAY_WITH_RNG)
	vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);
#endif
	
	portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
}

#else

void HAL_I2SEx_TxCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
    BaseType_t xHigherPriorityTaskWorken;
    wavtxintflag = 1;
    wavtxflag = 1;
    xHigherPriorityTaskWorken = pdFALSE;
#if defined(PLAY_WITH_LIST)
    vTaskNotifyGiveFromISR(audioplaywithlistTaskHandle,&xHigherPriorityTaskWorken);
#elif defined(PLAY_WITH_RNG)
    vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);
#endif
    portYIELD_FROM_ISR(xHigherPriorityTaskWorken);

}
void HAL_I2SEx_TxHalfCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
    wavtxintflag = 1;
	wavtxflag = 0;
	xHigherPriorityTaskWorken = pdFALSE;
#if defined(PLAY_WITH_LIST)
	vTaskNotifyGiveFromISR(audioplaywithlistTaskHandle,&xHigherPriorityTaskWorken);
#elif defined(PLAY_WITH_RNG)
	vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);
#endif

	portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
}
void HAL_I2SEx_RxCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
    BaseType_t xHigherPriorityTaskWorken;
    wavrxintflag = 1;
    wavrxflag = 1;
    xHigherPriorityTaskWorken = pdFALSE;
#if defined(PLAY_WITH_LIST)
    vTaskNotifyGiveFromISR(audioplaywithlistTaskHandle,&xHigherPriorityTaskWorken);
#elif defined(PLAY_WITH_RNG)
    vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);
#endif
    portYIELD_FROM_ISR(xHigherPriorityTaskWorken);

}
void HAL_I2SEx_RxHalfCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
    BaseType_t xHigherPriorityTaskWorken;
    wavrxintflag = 1;
    wavrxflag = 0;
    xHigherPriorityTaskWorken = pdFALSE;
#if defined(PLAY_WITH_LIST)
    vTaskNotifyGiveFromISR(audioplaywithlistTaskHandle,&xHigherPriorityTaskWorken);
#elif defined(PLAY_WITH_RNG)
    vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);
#endif

    portYIELD_FROM_ISR(xHigherPriorityTaskWorken);

}
#endif

