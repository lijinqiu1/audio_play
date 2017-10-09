#include <string.h>
#include "wav.h"
#include "ff.h"
#include "freertos.h"
#include "wm8978.h"
#include "i2s.h"
#include "fatfs.h"
#include "rng.h"
#include "gpio.h"
#include "main.h"

__wavctrl wavctrl;		//WAV���ƽṹ��
uint8_t wavtransferend=0;	//i2s������ɱ�־
uint8_t wavreceiveend=0;	//i2s������ɱ�־
uint8_t wavwitchrxbuf=0;		//i2sbufxָʾ��־
uint8_t wavwitchtxbuf=0;		//i2sbufxָʾ��־
uint8_t wavrxtxflag=0; //txrx����״̬��ʶ

osThreadId audioplayTaskHandle;

extern EventGroupHandle_t xEventGroup;
//���ֲ��ſ�����
__audiodev audiodev;
void HAL_I2S_M1_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_M1_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);

//*********************************¼������*****************************
uint8_t *i2srecbuf1;
uint8_t *i2srecbuf2;

uint32_t wavsize;  //wav���ݴ�С
uint8_t rec_sta = 0;  //¼��״̬

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

FRESULT wav_decode_init(uint8_t *fname, __wavctrl* wavx)
{
	FIL *ftemp;
	uint8_t *buf;
	uint32_t br = 0;
	FRESULT res = FR_OK;

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
	if(res == FR_OK)
	{
		goto error2;
	}
	res = f_read(ftemp,buf,512,&br); //��ȡ512�ֽ�����
    if (res == FR_OK)
    {
    	goto error1;
    }
    else
    {
        app_trace_log("error %d %d %s\n",res,__LINE__,__FUNCTION__);
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
uint32_t wav_buffill(uint8_t *buf,uint8_t *tbuf,FIL*file,uint16_t size,uint8_t bits)
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
void wav_get_curtime(FIL*fx,__wavctrl *wavx)
{
	long long fpos;
 	wavx->totsec=wavx->datasize/(wavx->bitrate/8);	//�����ܳ���(��λ:��)
	fpos=fx->fptr-wavx->datastart; 					//�õ���ǰ�ļ����ŵ��ĵط�
	wavx->cursec=fpos*wavx->totsec/wavx->datasize;	//��ǰ���ŵ��ڶ�������?
}

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


#if defined (USE_MULT_DMA)
/**
  * @brief DMA I2S transmit process complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMATxCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef* hi2s = ( I2S_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U)
  {
    /* Disable Tx DMA Request */
    CLEAR_BIT(hi2s->Instance->CR2,SPI_CR2_TXDMAEN);

    hi2s->TxXferCount = 0U;
    hi2s->State       = HAL_I2S_STATE_READY;
  }
  HAL_I2S_TxCpltCallback(hi2s);
}

/**
  * @brief DMA I2S transmit process complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_M1_DMATxCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef* hi2s = ( I2S_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  HAL_I2S_M1_TxCpltCallback(hi2s);
}

/**
  * @brief DMA I2S transmit process half complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef* hi2s = (I2S_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  HAL_I2S_TxHalfCpltCallback(hi2s);
}

/**
  * @brief DMA I2S transmit process half complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_M1_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef* hi2s = (I2S_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  HAL_I2S_M1_TxHalfCpltCallback(hi2s);
}

/**
  * @brief DMA I2S communication error callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMAError(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef* hi2s = (I2S_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  /* Disable Rx and Tx DMA Request */
  CLEAR_BIT(hi2s->Instance->CR2,(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN));
  hi2s->TxXferCount = 0U;
  hi2s->RxXferCount = 0U;

  hi2s->State= HAL_I2S_STATE_READY;

  SET_BIT(hi2s->ErrorCode,HAL_I2S_ERROR_DMA);
  HAL_I2S_ErrorCallback(hi2s);
}

static HAL_StatusTypeDef HAL_I2S_Transmit_DMA_MultiBufferStart(I2S_HandleTypeDef *hi2s, uint16_t *pData,uint16_t *pData2, uint16_t Size)
{
  uint32_t *tmp = NULL;
  uint32_t *tmp2 = NULL;
  uint32_t tmp1 = 0U;

  if((pData == NULL) || (Size == 0U))
  {
    return  HAL_ERROR;
  }

  if(hi2s->State == HAL_I2S_STATE_READY)
  {
    hi2s->pTxBuffPtr = pData;
    tmp1 = hi2s->Instance->I2SCFGR & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN);
    if((tmp1 == I2S_DATAFORMAT_24B) || (tmp1 == I2S_DATAFORMAT_32B))
    {
      hi2s->TxXferSize  = (Size << 1U);
      hi2s->TxXferCount = (Size << 1U);
    }
    else
    {
      hi2s->TxXferSize  = Size;
      hi2s->TxXferCount = Size;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->State     = HAL_I2S_STATE_BUSY_TX;

    /* Set the I2S Tx DMA Half transfer complete callback */
    hi2s->hdmatx->XferHalfCpltCallback = I2S_DMATxHalfCplt;

    hi2s->hdmatx->XferM1HalfCpltCallback = I2S_M1_DMATxHalfCplt;

    /* Set the I2S Tx DMA transfer complete callback */
    hi2s->hdmatx->XferCpltCallback = I2S_DMATxCplt;

    hi2s->hdmatx->XferM1CpltCallback = I2S_M1_DMATxCplt;

    /* Set the DMA error callback */
    hi2s->hdmatx->XferErrorCallback = I2S_DMAError;

    /* Enable the Tx DMA Stream */
    tmp = (uint32_t*)&pData;
    tmp2 = (uint32_t*)&pData2;
    HAL_DMAEx_MultiBufferStart_IT(hi2s->hdmatx, *(uint32_t*)tmp, (uint32_t)&hi2s->Instance->DR,*(uint32_t*)tmp2, hi2s->TxXferSize);

    /* Check if the I2S is already enabled */
    if((hi2s->Instance->I2SCFGR &SPI_I2SCFGR_I2SE) != SPI_I2SCFGR_I2SE)
    {
      /* Enable I2S peripheral */
      __HAL_I2S_ENABLE(hi2s);
    }

     /* Check if the I2S Tx request is already enabled */
    if((hi2s->Instance->CR2 & SPI_CR2_TXDMAEN) != SPI_CR2_TXDMAEN)
    {
      /* Enable Tx DMA Request */
      SET_BIT(hi2s->Instance->CR2, SPI_CR2_TXDMAEN);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
#endif
//����ĳ��WAV�ļ�
//fname:wav�ļ�·��.
//����ֵ:
//KEY0_PRES:��һ��
//KEY1_PRES:��һ��
//����:����
uint8_t wav_play_song(uint8_t* fname)
{
	FRESULT res;
	uint32_t fillnum;
	uint32_t ulEventsToProcess;
	//��Ƶ�ļ����
	audiodev.file1=(FIL*)pvPortMalloc(sizeof(FIL));
	//��Ƶ������
	audiodev.i2sbuf1=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);
	//�����������ʱʹ��
	audiodev.tbuf=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);

	if(!audiodev.file1||!audiodev.i2sbuf1||!audiodev.tbuf)
	{
		goto error2;
	}
	res = wav_decode_init(fname,&wavctrl);//�õ��ļ�����Ϣ
	if(res != FR_OK)//�����ļ��ɹ�
	{
		goto error2;
	}
	if(wavctrl.bps==16)
	{
		WM8978_I2S_Cfg(2,0);	//�����ֱ�׼,16λ���ݳ���
	}
    else if(wavctrl.bps==24)
	{
		WM8978_I2S_Cfg(2,2);	//�����ֱ�׼,24λ���ݳ���
	}
    IIS_Init(wavctrl.bps,wavctrl.samplerate); //IISʱ�ӳ�ʼ��
	res=f_open(audiodev.file1,(const TCHAR*)fname,FA_READ);	//���ļ�
	if(res!=FR_OK)
	{
		goto error2;
	}
	res = f_lseek(audiodev.file1, wavctrl.datastart);		//�����ļ�ͷ
    if (res != FR_OK)
    {
    	goto error1;
    }
    fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE,wavctrl.bps);
	if (fillnum != WAV_I2S_TX_DMA_BUFSIZE/2)
	{
		goto error1;
	}
	HAL_I2S_Transmit_DMA(&hi2s2,(uint16_t *)audiodev.i2sbuf1,WAV_I2S_TX_DMA_BUFSIZE/2);
    fillnum = WAV_I2S_TX_DMA_BUFSIZE/2;

    while(1)
    {
    	ulEventsToProcess = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		if (ulEventsToProcess != 0)
		{

        	if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//���Ž���
        	{
	            app_trace_log("fillnum %d %d %s\n",fillnum,__LINE__,__FUNCTION__);
	            break;
        	}
	        if(wavwitchtxbuf) {
            	fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//���buf1
	        }
        	else {
            	fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//���buf1
        	}
        	wav_get_curtime(audiodev.file1,&wavctrl);
    	}
    }
error1:
	f_close(audiodev.file1);
error2:
	vPortFree(audiodev.i2sbuf1);
	vPortFree(audiodev.tbuf);
	vPortFree(audiodev.file1);
	return res;
}

//����PCM ¼��ģʽ
void recoder_enter_rec_mode(uint32_t DataFormat,uint32_t AudioFreq)
{
//    WM8978_ADDA_Cfg(0,1);		//����ADC
//	WM8978_Input_Cfg(1,0,0);	//��������ͨ��(MIC&LINE IN)
//	WM8978_Output_Cfg(0,1);		//����BYPASS���
//	WM8978_MIC_Gain(46);		//MIC��������

    IIS_Init(DataFormat,AudioFreq);
	
}

//ͨ��ʱ���ȡ�ļ���
//������SD������,��֧��FLASH DISK����
//��ϳ�:����"0:RECORDER/REC20120321210633.wav"���ļ���
void recoder_new_pathname(uint8_t *pname)
{
	uint8_t res;
	uint16_t index=0;
    FIL fil;
	while(index<0XFFFF)
	{
		sprintf((char*)pname,"0:RECORDER/REC%05d.wav",index);
		res=f_open(&fil,(const TCHAR*)pname,FA_READ);//���Դ�����ļ�
		if(res==FR_NO_FILE)break;		//���ļ���������=����������Ҫ��.
		index++;
        f_close(&fil);
	}
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

//¼������
void wav_recorder(void)
{
    FRESULT res;
    DIR recdir;
    uint8_t *pname;
    uint32_t bw;
    uint32_t tick;
	//֪ͨ
	uint32_t ulEventsToProcess;
	__WaveHeader *wavhead=0;
    while(f_opendir(&recdir,"0:/RECORDER"))//��¼���ļ���
    {
        res = f_mkdir("0:/RECORDER");
        if (res != FR_OK)
        {
            app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
            return ;
        }
    }
	//�����ڴ�
    audiodev.file2=(FIL*)pvPortMalloc(sizeof(FIL));
	audiodev.i2sbuf1=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);
	audiodev.i2sbuf2=pvPortMalloc(WAV_I2S_RX_DMA_BUFSIZE);
	//����wav���ļ�ͷ�ڴ�
    wavhead = (__WaveHeader*)pvPortMalloc(sizeof(__WaveHeader));
	//�����ļ����ڴ�
    pname = pvPortMalloc(30);
    if(!audiodev.file2||!audiodev.i2sbuf1||!audiodev.i2sbuf2||!wavhead||!pname)
    {
    	goto error2;
    }
	memset(audiodev.i2sbuf1,0x00,WAV_I2S_TX_DMA_BUFSIZE);
    pname[0]= 0;
	//��ȡ�µ�¼���ļ���
    recoder_new_pathname(pname);
    app_trace_log("recorder: %s\n",pname);
	//�ļ�ͷ��ʼ��
    recoder_wav_init(wavhead,16,I2S_AUDIOFREQ_16K);
	//����¼��ģʽ����ʱ��������������ͷ�ɼ�������Ƶ
    recoder_enter_rec_mode(16,I2S_AUDIOFREQ_16K);
	WM8978_I2S_Cfg(2,0);		//�����ֱ�׼,16λ���ݳ���
	HAL_I2SEx_TransmitReceive_DMA_A(&hi2s2,(uint16_t *)audiodev.i2sbuf1,(uint16_t *)audiodev.i2sbuf2,WAV_I2S_TX_DMA_BUFSIZE/2);

    res=f_open(audiodev.file2,(const TCHAR*)pname,FA_CREATE_ALWAYS|FA_WRITE);
    if (res != FR_OK)
    {
		app_trace_log("error %d, %s, %d\n",res, __FUNCTION__, __LINE__);
    	goto error2;
    }
    res= f_write(audiodev.file2,(const void*)wavhead,sizeof(__WaveHeader),&bw);
	if (res != FR_OK)
	{
		app_trace_log("error %d, %s, %d\n",res, __FUNCTION__, __LINE__);
		goto error1;
	}
    tick = HAL_GetTick();
    rec_sta=1;
    while(HAL_GetTick() - tick < 10000)//¼����ʼ
    {
		ulEventsToProcess = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		if (ulEventsToProcess != 0)
		{
#if 0
            if(wavwitchrxbuf)
            {
                res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
                if(res != FR_OK)
                {
                    app_trace_log("write error:%d\r\n",res);
					break;
                }
                else
                {
                    wavsize+=WAV_I2S_TX_DMA_BUFSIZE/2;
				}
			}
            else
            {
                res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
                if(res != FR_OK)
                {
                    app_trace_log("write error:%d\r\n",res);
					break;
                }
                else
                {
                    wavsize+=WAV_I2S_TX_DMA_BUFSIZE/2;
				}
            }
#else
			if(wavrxtxflag)
            {
                res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
                if(res != FR_OK)
                {
                    app_trace_log("write error:%d\r\n",res);
					break;
                }
                else
                {
                    wavsize+=WAV_I2S_TX_DMA_BUFSIZE/2;
				}
			}
            else
            {
                res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//д���ļ�
                if(res != FR_OK)
                {
                    app_trace_log("write error:%d\r\n",res);
					break;
                }
                else
                {
                    wavsize+=WAV_I2S_TX_DMA_BUFSIZE/2;
				}
            }
#endif
		}
    }
    rec_sta=0;	//�ر�¼��
    wavhead->riff.ChunkSize=wavsize+36;		//�����ļ��Ĵ�С-8;
    wavhead->data.ChunkSize=wavsize;		//���ݴ�С
    res= f_lseek(audiodev.file2,0);						//ƫ�Ƶ��ļ�ͷ.
    if (res != FR_OK)
    {
        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
        goto error1;
    }
    res= f_write(audiodev.file2,(const void*)wavhead,sizeof(__WaveHeader),&bw);//д��ͷ����
    if (res != FR_OK)
    {
        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
        goto error1;
    }
error1:
    res= f_close(audiodev.file2);
    if (res != FR_OK)
    {
        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
    }
    app_trace_log("recorder end\n");
error2:
    wavsize=0;
    vPortFree(audiodev.i2sbuf1);
    vPortFree(audiodev.i2sbuf2);
    vPortFree(audiodev.file2);
    vPortFree(wavhead);
    vPortFree(pname);

}

//ͬʱ����¼������
void wav_recorder_play(uint8_t* fname)
{
	FRESULT res;
	DIR recdir;
	uint8_t *pname;
	uint32_t bw;
	uint32_t fillnum;
	uint32_t ulEventsToProcess;
	__WaveHeader *wavheadrx=0;
	while(f_opendir(&recdir,"0:/RECORDER"))//��¼���ļ���
	{
		res = f_mkdir("0:/RECORDER");
		if (res != FR_OK)
		{
			app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			return ;
		}
	}
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
	pname = pvPortMalloc(30);
	//�õ��ļ�����Ϣ
	res = wav_decode_init(fname,&wavctrl);
	if (res != FR_OK)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	if(wavctrl.bps==16)
	{
		WM8978_I2S_Cfg(2,0);	//�����ֱ�׼,16λ���ݳ���
	}
    else if(wavctrl.bps==24)
	{
		WM8978_I2S_Cfg(2,2);	//�����ֱ�׼,24λ���ݳ���
	}

	IIS_Init(wavctrl.bps,wavctrl.samplerate);
	if(!audiodev.file1||!audiodev.file2||!audiodev.i2sbuf1||!audiodev.i2sbuf2||!audiodev.tbuf ||\
		!wavheadrx||!pname)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	res=f_open(audiodev.file1,(const TCHAR*)fname,FA_READ);	//���ļ�
	if (res != FR_OK)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	res = f_lseek(audiodev.file1, wavctrl.datastart);		//�����ļ�ͷ
	if (res != FR_OK)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error1;
	}
	pname[0]= 0;
	//��ȡ¼���ļ���
    recoder_new_pathname(pname);
    app_trace_log("recorder: %s\n",pname);
	//��ʼ��¼���ļ�ͷ
    recoder_wav_init(wavheadrx,wavctrl.bps,wavctrl.samplerate);
    res=f_open(audiodev.file2,(const TCHAR*)pname,FA_CREATE_ALWAYS|FA_WRITE);
	if (res != FR_OK)
	{
		app_trace_log("error:%d ,%s,%d\n",res,__FUNCTION__,__LINE__);
		goto error2;
	}
    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);
    if (res != FR_OK)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error1;
	}
	fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE,wavctrl.bps);
	fillnum = WAV_I2S_TX_DMA_BUFSIZE/2;
	HAL_I2SEx_TransmitReceive_DMA_A(&hi2s2,(uint16_t *)audiodev.i2sbuf1,(uint16_t *)audiodev.i2sbuf2,\
		WAV_I2S_TX_DMA_BUFSIZE/2);
	//��ʼ¼������
    rec_sta=1;
	while(1)
	{
#if 0
		if(wavtransferend)
		{
			wavtransferend = 0;
			if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//���Ž���?
			{
				app_trace_log("play end\n");
				break;
			}
			if(wavwitchtxbuf)
                fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),WAV_I2S_TX_DMA_BUFSIZE/2,\
                					wavctrl.bps);//���buf1
            else
                fillnum=wav_buffill(audiodev.i2sbuf1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//���buf1
		}
		if(wavreceiveend)
		{
			wavreceiveend = 0;
			if(wavwitchrxbuf)
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
            else
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
#else
		ulEventsToProcess = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		if (ulEventsToProcess != 0)
		{
			if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//���Ž���?
			{
				app_trace_log("play end\n");
				break;
			}
			if (wavrxtxflag)
			{
				fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,\
                					wavctrl.bps);//���buf1
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
			else
			{
				fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//���buf1
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
#endif
	}
	rec_sta=0;	//�ر�¼��
    wavheadrx->riff.ChunkSize=wavsize+36;		//�����ļ��Ĵ�С-8;
    wavheadrx->data.ChunkSize=wavsize;		//���ݴ�С
    res= f_lseek(audiodev.file2,0);						//ƫ�Ƶ��ļ�ͷ.
    if (res != FR_OK)
    {
        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
        goto error1;
    }
    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);//д��ͷ����
    if (res != FR_OK)
    {
        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
        goto error1;
    }
error1:
    res= f_close(audiodev.file1);
    if (res != FR_OK)
    {
        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
    }
    app_trace_log("recorder end\n");
error2:
    wavsize=0;
    vPortFree(audiodev.i2sbuf1);
    vPortFree(audiodev.i2sbuf2);
	vPortFree(audiodev.tbuf);
    vPortFree(audiodev.file1);
    vPortFree(audiodev.file2);
    vPortFree(wavheadrx);
    vPortFree(pname);
}

//*******************************************************************************
//��ȡ�����б� ���������ļ�����
//*******************************************************************************
uint8_t Get_Play_List(void)
{
	DIR recdir;
	FRESULT res;
	FILINFO wavfileinfo;
	uint8_t file_count = 0;
	res = f_opendir(&recdir,"0:/RECORDER");
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
		file_count ++ ;
		app_trace_log("%s\r\n",wavfileinfo.fname);
	}

error:
	return file_count;
}

void Get_Play_Song(uint8_t *fname,uint8_t file_count)
{
    DIR recdir;
	FRESULT res;
	FILINFO fileinfo;
	uint32_t number;
	uint8_t i;

	number = HAL_RNG_GetRandomNumber(&hrng);
	
	number = number % file_count;
	res = f_opendir(&recdir,"0:/MUSIC");
	if (res != FR_OK)
	{
		return ;
	}
	
	for(i = 0;i < number; i++)
	{
		res = f_readdir(&recdir,&fileinfo);
		if (res != FR_OK)
		{
			fname[0] = 0;
			break;
		}
		if (fileinfo.fname[0] == 0)
		{
			fname[0] = 0;
			break;
		}
	}
	strcpy((char*)fname,fileinfo.fname);
}
//*******************************************************************************
//��Ƶ����¼���߳�
//*******************************************************************************
void AudioPlay_Task(void const * argument)
{
	//��ȡmusic�ļ�������Ƶ�ļ�
	FATFS fs;
	FRESULT res;
	EventBits_t xEventGroupValue;
	DIR recdir;
	uint8_t file_count;
	uint8_t fname[13];
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
	//wav�ļ�ͷ
	__WaveHeader *wavheadrx;
	const EventBits_t xBitsToWaitFor = ( KEY_VOL_UP_BIT |
	                                     KEY_VOL_DOWN_BIT |
	                                     KEY_FUN_STOP_BIT |
	                                     KEY_PLAY_BIT |
	                                     KEY_RECORD_BIT |
	                                     KEY_PLAY_AND_RECORD_BIT);
	const EventBits_t uxAllSyncBits = ( KEY_VOL_UP_BIT |
										KEY_VOL_DOWN_BIT |
										KEY_FUN_STOP_BIT |
										KEY_PLAY_BIT |
										KEY_RECORD_BIT |
										KEY_PLAY_AND_RECORD_BIT |
										KEY_ASK_BIT |
										KEY_FUN_BLE_BIT);
	//����SD��
    res = f_mount(&fs,(const TCHAR*)SD_Path,0);
    if (res !=FR_OK)
    {
    	 goto error2;
	}
	file_count = Get_Play_List();
	while(f_opendir(&recdir,"0:/RECORDER"))//��¼���ļ���
	{
		res = f_mkdir("0:/RECORDER");
		if (res != FR_OK)
		{
			app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			goto error2 ;
		}
	}	
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
	pname = pvPortMalloc(30);
	if(!audiodev.file1 || !audiodev.file2 || !audiodev.i2sbuf1 || !audiodev.i2sbuf2 || !audiodev.tbuf ||\
			!wavheadrx || !pname || !wavheadrx)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	//��ʼ�������ʡ�����λ��
	wavctrl.bps = 16;
	wavctrl.samplerate = I2S_AUDIOFREQ_16K;
	while(1)
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
		xEventGroupSync(xEventGroup,
						xBitsToWaitFor,
						uxAllSyncBits,
						portMAX_DELAY);

		
		if((xEventGroupValue & KEY_PLAY_BIT)!=0 || (xEventGroupValue & KEY_PLAY_AND_RECORD_BIT)!=0)
		{//���Ų��ֳ�ʼ��
			play_begin = 1;
			//��ȥ��������ļ���
			Get_Play_Song(fname,file_count);
			//�õ��ļ�����Ϣ
			res = wav_decode_init(fname,&wavctrl);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error2;
			}
			//�򿪲����ļ�
			res=f_open(audiodev.file1,(const TCHAR*)fname,FA_READ);	
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
		}
		
		if((xEventGroupValue & KEY_PLAY_AND_RECORD_BIT)!=0 || (xEventGroupValue & KEY_RECORD_BIT)!=0)
		{//¼�����ֳ�ʼ��
			record_begin = 1;
			//��ȡ¼���ļ���
		    recoder_new_pathname(pname);
		    app_trace_log("recorder: %s\n",pname);
			//��ʼ��¼���ļ�ͷ
		    recoder_wav_init(wavheadrx,wavctrl.bps,wavctrl.samplerate);
			//��¼���ļ�
		    res=f_open(audiodev.file2,(const TCHAR*)pname,FA_CREATE_ALWAYS|FA_WRITE);
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
		}
		
		if ((xEventGroupValue & KEY_PLAY_BIT)!=0 || (xEventGroupValue & KEY_PLAY_AND_RECORD_BIT)!=0 ||
			(xEventGroupValue & KEY_RECORD_BIT)!=0)
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
		    rec_sta=1;
		}
		
		if ((xEventGroupValue & KEY_FUN_STOP_BIT) != 0)
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
				if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//���Ž���?
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
				    wavheadrx->data.ChunkSize=wavsize;		//���ݴ�С
				    res= f_lseek(audiodev.file2,0);						//ƫ�Ƶ��ļ�ͷ.
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				        goto error1;
				    }
				    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);//д��ͷ����
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				        goto error1;
				    }
				}
				if (play_begin == 1)
				{
				    res= f_close(audiodev.file1);
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				    }
				}
			    app_trace_log("%s,%d,end\n",__FUNCTION__,__LINE__);
    			wavsize=0;
				rec_sta=0;	//�ر��жϷ���
				play_begin = 0; //�رղ���
				record_begin = 0; //�ر�¼��
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
	while(1)
	{
		osDelay(1000);
	}
}
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);
  BaseType_t xHigherPriorityTaskWorken;
  wavwitchtxbuf = 1;
  xHigherPriorityTaskWorken = pdFALSE;

  vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2S_TxCpltCallback could be implemented in the user file
   */
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);
  BaseType_t xHigherPriorityTaskWorken;
  wavwitchtxbuf = 0;
  xHigherPriorityTaskWorken = pdFALSE;

  vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2S_TxCpltCallback could be implemented in the user file
   */
}

#if defined (USE_MULT_DMA)
void HAL_I2S_M1_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);
  wavwitchbuf = 0;
  wavtransferend=1;
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2S_TxCpltCallback could be implemented in the user file
   */
}

void HAL_I2S_M1_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2S_TxCpltCallback could be implemented in the user file
   */
}
#endif
void HAL_I2SEx_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
    if(rec_sta){
		wavtransferend = 1;
        wavwitchrxbuf=0;		//i2sbufxָʾ��־
		xHigherPriorityTaskWorken = pdFALSE;

		vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
    }
}
void HAL_I2SEx_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
    if (rec_sta){
		wavtransferend = 1;
		wavwitchrxbuf=1;		//i2sbufxָʾ��־
		xHigherPriorityTaskWorken = pdFALSE;

		vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
    }
}

void HAL_I2SEx_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    UNUSED(hi2s);

    wavtransferend=1;	//i2s������ɱ�־
    wavwitchtxbuf=0;		//i2sbufxָʾ��־
}
void HAL_I2SEx_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    UNUSED(hi2s);

	wavtransferend=1;	//i2s������ɱ�־
	wavwitchtxbuf=1;		//i2sbufxָʾ��־
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
	if (rec_sta){
		wavrxtxflag = 0;
		xHigherPriorityTaskWorken = pdFALSE;

		vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
	}

}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef * hi2s)
{
    UNUSED(hi2s);
	BaseType_t xHigherPriorityTaskWorken;
    if (rec_sta){
		wavrxtxflag = 1;
		xHigherPriorityTaskWorken = pdFALSE;

		vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
    }
}

