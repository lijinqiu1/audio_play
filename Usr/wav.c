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

__wavctrl wavctrl;		//WAV控制结构体
uint8_t wavtransferend=0;	//i2s传输完成标志
uint8_t wavreceiveend=0;	//i2s传输完成标志
uint8_t wavwitchrxbuf=0;		//i2sbufx指示标志
uint8_t wavwitchtxbuf=0;		//i2sbufx指示标志
uint8_t wavrxtxflag=0; //txrx传输状态标识

osThreadId audioplayTaskHandle;

extern EventGroupHandle_t xEventGroup;
//音乐播放控制器
__audiodev audiodev;
void HAL_I2S_M1_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_M1_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);

//*********************************录音部分*****************************
uint8_t *i2srecbuf1;
uint8_t *i2srecbuf2;

uint32_t wavsize;  //wav数据大小
uint8_t rec_sta = 0;  //录音状态

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
	if(!ftemp || !buf) //内存申请成功
	{
		goto error2;
	}
	app_trace_log("malloc success!\n");
	res = f_open(ftemp,(const TCHAR*)fname,FA_READ);
	if(res == FR_OK)
	{
		goto error2;
	}
	res = f_read(ftemp,buf,512,&br); //读取512字节数据
    if (res == FR_OK)
    {
    	goto error1;
    }
    else
    {
        app_trace_log("error %d %d %s\n",res,__LINE__,__FUNCTION__);
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
uint32_t wav_buffill(uint8_t *buf,uint8_t *tbuf,FIL*file,uint16_t size,uint8_t bits)
{
	uint16_t readlen=0;
	uint32_t bread = 0;
	uint16_t i = 0;
	uint8_t *p;
    FRESULT res;
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
	}
	return bread;
}

//得到当前播放时间
//fx:文件指针
//wavx:wav播放控制器
void wav_get_curtime(FIL*fx,__wavctrl *wavx)
{
	long long fpos;
 	wavx->totsec=wavx->datasize/(wavx->bitrate/8);	//歌曲总长度(单位:秒)
	fpos=fx->fptr-wavx->datastart; 					//得到当前文件播放到的地方
	wavx->cursec=fpos*wavx->totsec/wavx->datasize;	//当前播放到第多少秒了?
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
//播放某个WAV文件
//fname:wav文件路径.
//返回值:
//KEY0_PRES:下一曲
//KEY1_PRES:上一曲
//其他:错误
uint8_t wav_play_song(uint8_t* fname)
{
	FRESULT res;
	uint32_t fillnum;
	uint32_t ulEventsToProcess;
	//音频文件句柄
	audiodev.file1=(FIL*)pvPortMalloc(sizeof(FIL));
	//音频数据域
	audiodev.i2sbuf1=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);
	//用于数据填充时使用
	audiodev.tbuf=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);

	if(!audiodev.file1||!audiodev.i2sbuf1||!audiodev.tbuf)
	{
		goto error2;
	}
	res = wav_decode_init(fname,&wavctrl);//得到文件的信息
	if(res != FR_OK)//解析文件成功
	{
		goto error2;
	}
	if(wavctrl.bps==16)
	{
		WM8978_I2S_Cfg(2,0);	//飞利浦标准,16位数据长度
	}
    else if(wavctrl.bps==24)
	{
		WM8978_I2S_Cfg(2,2);	//飞利浦标准,24位数据长度
	}
    IIS_Init(wavctrl.bps,wavctrl.samplerate); //IIS时钟初始化
	res=f_open(audiodev.file1,(const TCHAR*)fname,FA_READ);	//打开文件
	if(res!=FR_OK)
	{
		goto error2;
	}
	res = f_lseek(audiodev.file1, wavctrl.datastart);		//跳过文件头
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

        	if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//播放结束
        	{
	            app_trace_log("fillnum %d %d %s\n",fillnum,__LINE__,__FUNCTION__);
	            break;
        	}
	        if(wavwitchtxbuf) {
            	fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//填充buf1
	        }
        	else {
            	fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//填充buf1
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

//进入PCM 录音模式
void recoder_enter_rec_mode(uint32_t DataFormat,uint32_t AudioFreq)
{
//    WM8978_ADDA_Cfg(0,1);		//开启ADC
//	WM8978_Input_Cfg(1,0,0);	//开启输入通道(MIC&LINE IN)
//	WM8978_Output_Cfg(0,1);		//开启BYPASS输出
//	WM8978_MIC_Gain(46);		//MIC增益设置

    IIS_Init(DataFormat,AudioFreq);
	
}

//通过时间获取文件名
//仅限在SD卡保存,不支持FLASH DISK保存
//组合成:形如"0:RECORDER/REC20120321210633.wav"的文件名
void recoder_new_pathname(uint8_t *pname)
{
	uint8_t res;
	uint16_t index=0;
    FIL fil;
	while(index<0XFFFF)
	{
		sprintf((char*)pname,"0:RECORDER/REC%05d.wav",index);
		res=f_open(&fil,(const TCHAR*)pname,FA_READ);//尝试打开这个文件
		if(res==FR_NO_FILE)break;		//该文件名不存在=正是我们需要的.
		index++;
        f_close(&fil);
	}
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

//录音程序
void wav_recorder(void)
{
    FRESULT res;
    DIR recdir;
    uint8_t *pname;
    uint32_t bw;
    uint32_t tick;
	//通知
	uint32_t ulEventsToProcess;
	__WaveHeader *wavhead=0;
    while(f_opendir(&recdir,"0:/RECORDER"))//打开录音文件夹
    {
        res = f_mkdir("0:/RECORDER");
        if (res != FR_OK)
        {
            app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
            return ;
        }
    }
	//申请内存
    audiodev.file2=(FIL*)pvPortMalloc(sizeof(FIL));
	audiodev.i2sbuf1=pvPortMalloc(WAV_I2S_TX_DMA_BUFSIZE);
	audiodev.i2sbuf2=pvPortMalloc(WAV_I2S_RX_DMA_BUFSIZE);
	//申请wav把文件头内存
    wavhead = (__WaveHeader*)pvPortMalloc(sizeof(__WaveHeader));
	//申请文件名内存
    pname = pvPortMalloc(30);
    if(!audiodev.file2||!audiodev.i2sbuf1||!audiodev.i2sbuf2||!wavhead||!pname)
    {
    	goto error2;
    }
	memset(audiodev.i2sbuf1,0x00,WAV_I2S_TX_DMA_BUFSIZE);
    pname[0]= 0;
	//获取新的录音文件名
    recoder_new_pathname(pname);
    app_trace_log("recorder: %s\n",pname);
	//文件头初始化
    recoder_wav_init(wavhead,16,I2S_AUDIOFREQ_16K);
	//进入录音模式，此时耳机可以听到咪头采集到的音频
    recoder_enter_rec_mode(16,I2S_AUDIOFREQ_16K);
	WM8978_I2S_Cfg(2,0);		//飞利浦标准,16位数据长度
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
    while(HAL_GetTick() - tick < 10000)//录音开始
    {
		ulEventsToProcess = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		if (ulEventsToProcess != 0)
		{
#if 0
            if(wavwitchrxbuf)
            {
                res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
                res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
                res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
                res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
    rec_sta=0;	//关闭录音
    wavhead->riff.ChunkSize=wavsize+36;		//整个文件的大小-8;
    wavhead->data.ChunkSize=wavsize;		//数据大小
    res= f_lseek(audiodev.file2,0);						//偏移到文件头.
    if (res != FR_OK)
    {
        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
        goto error1;
    }
    res= f_write(audiodev.file2,(const void*)wavhead,sizeof(__WaveHeader),&bw);//写入头数据
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

//同时播放录音函数
void wav_recorder_play(uint8_t* fname)
{
	FRESULT res;
	DIR recdir;
	uint8_t *pname;
	uint32_t bw;
	uint32_t fillnum;
	uint32_t ulEventsToProcess;
	__WaveHeader *wavheadrx=0;
	while(f_opendir(&recdir,"0:/RECORDER"))//打开录音文件夹
	{
		res = f_mkdir("0:/RECORDER");
		if (res != FR_OK)
		{
			app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			return ;
		}
	}
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
	pname = pvPortMalloc(30);
	//得到文件的信息
	res = wav_decode_init(fname,&wavctrl);
	if (res != FR_OK)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	if(wavctrl.bps==16)
	{
		WM8978_I2S_Cfg(2,0);	//飞利浦标准,16位数据长度
	}
    else if(wavctrl.bps==24)
	{
		WM8978_I2S_Cfg(2,2);	//飞利浦标准,24位数据长度
	}

	IIS_Init(wavctrl.bps,wavctrl.samplerate);
	if(!audiodev.file1||!audiodev.file2||!audiodev.i2sbuf1||!audiodev.i2sbuf2||!audiodev.tbuf ||\
		!wavheadrx||!pname)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	res=f_open(audiodev.file1,(const TCHAR*)fname,FA_READ);	//打开文件
	if (res != FR_OK)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	res = f_lseek(audiodev.file1, wavctrl.datastart);		//跳过文件头
	if (res != FR_OK)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error1;
	}
	pname[0]= 0;
	//获取录音文件名
    recoder_new_pathname(pname);
    app_trace_log("recorder: %s\n",pname);
	//初始化录音文件头
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
	//开始录音播放
    rec_sta=1;
	while(1)
	{
#if 0
		if(wavtransferend)
		{
			wavtransferend = 0;
			if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//播放结束?
			{
				app_trace_log("play end\n");
				break;
			}
			if(wavwitchtxbuf)
                fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),WAV_I2S_TX_DMA_BUFSIZE/2,\
                					wavctrl.bps);//填充buf1
            else
                fillnum=wav_buffill(audiodev.i2sbuf1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//填充buf1
		}
		if(wavreceiveend)
		{
			wavreceiveend = 0;
			if(wavwitchrxbuf)
            {
                res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
                res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
			if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//播放结束?
			{
				app_trace_log("play end\n");
				break;
			}
			if (wavrxtxflag)
			{
				fillnum=wav_buffill(audiodev.i2sbuf1+(WAV_I2S_TX_DMA_BUFSIZE/2),audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,\
                					wavctrl.bps);//填充buf1
                res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
				fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//填充buf1
				res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
	rec_sta=0;	//关闭录音
    wavheadrx->riff.ChunkSize=wavsize+36;		//整个文件的大小-8;
    wavheadrx->data.ChunkSize=wavsize;		//数据大小
    res= f_lseek(audiodev.file2,0);						//偏移到文件头.
    if (res != FR_OK)
    {
        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
        goto error1;
    }
    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);//写入头数据
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
//获取播放列表 返回音乐文件个数
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
//音频播放录音线程
//*******************************************************************************
void AudioPlay_Task(void const * argument)
{
	//获取music文件夹下音频文件
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
	//开始播放
	uint8_t play_begin = 0;
	//开始录音
	uint8_t record_begin = 0;
	//停止音频工作
	uint8_t stop_play_record = 0;
	//wav文件头
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
	//挂载SD卡
    res = f_mount(&fs,(const TCHAR*)SD_Path,0);
    if (res !=FR_OK)
    {
    	 goto error2;
	}
	file_count = Get_Play_List();
	while(f_opendir(&recdir,"0:/RECORDER"))//打开录音文件夹
	{
		res = f_mkdir("0:/RECORDER");
		if (res != FR_OK)
		{
			app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
			goto error2 ;
		}
	}	
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
	pname = pvPortMalloc(30);
	if(!audiodev.file1 || !audiodev.file2 || !audiodev.i2sbuf1 || !audiodev.i2sbuf2 || !audiodev.tbuf ||\
			!wavheadrx || !pname || !wavheadrx)
	{
		app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
		goto error2;
	}
	//初始化采样率、采样位数
	wavctrl.bps = 16;
	wavctrl.samplerate = I2S_AUDIOFREQ_16K;
	while(1)
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
		xEventGroupSync(xEventGroup,
						xBitsToWaitFor,
						uxAllSyncBits,
						portMAX_DELAY);

		
		if((xEventGroupValue & KEY_PLAY_BIT)!=0 || (xEventGroupValue & KEY_PLAY_AND_RECORD_BIT)!=0)
		{//播放部分初始化
			play_begin = 1;
			//过去随机播放文件名
			Get_Play_Song(fname,file_count);
			//得到文件的信息
			res = wav_decode_init(fname,&wavctrl);
			if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error2;
			}
			//打开播放文件
			res=f_open(audiodev.file1,(const TCHAR*)fname,FA_READ);	
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
		}
		
		if((xEventGroupValue & KEY_PLAY_AND_RECORD_BIT)!=0 || (xEventGroupValue & KEY_RECORD_BIT)!=0)
		{//录音部分初始化
			record_begin = 1;
			//获取录音文件名
		    recoder_new_pathname(pname);
		    app_trace_log("recorder: %s\n",pname);
			//初始化录音文件头
		    recoder_wav_init(wavheadrx,wavctrl.bps,wavctrl.samplerate);
			//打开录音文件
		    res=f_open(audiodev.file2,(const TCHAR*)pname,FA_CREATE_ALWAYS|FA_WRITE);
			if (res != FR_OK)
			{
				app_trace_log("error:%d ,%s,%d\n",res,__FUNCTION__,__LINE__);
				continue;
			}
			//写入文件头
		    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);
		    if (res != FR_OK)
			{
				app_trace_log("error %s,%d\n",__FUNCTION__,__LINE__);
				goto error1;
			}
		}
		
		if ((xEventGroupValue & KEY_PLAY_BIT)!=0 || (xEventGroupValue & KEY_PLAY_AND_RECORD_BIT)!=0 ||
			(xEventGroupValue & KEY_RECORD_BIT)!=0)
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
		    rec_sta=1;
		}
		
		if ((xEventGroupValue & KEY_FUN_STOP_BIT) != 0)
		{//停止播放
			stop_play_record = 1;
			goto end;
		}
		
		//开始录音播放
		if ((play_begin == 1) || (record_begin == 1))
		{	//等待中断
			ulEventsToProcess = ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
			if (ulEventsToProcess != 0)
			{
				if(fillnum!=WAV_I2S_TX_DMA_BUFSIZE/2)//播放结束?
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
	                	res=f_write(audiodev.file2,audiodev.i2sbuf2+WAV_I2S_RX_DMA_BUFSIZE/2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
						fillnum=wav_buffill(audiodev.i2sbuf1,audiodev.tbuf,audiodev.file1,WAV_I2S_TX_DMA_BUFSIZE/2,wavctrl.bps);//填充buf1
					}
					if(record_begin == 1)
					{
						res=f_write(audiodev.file2,audiodev.i2sbuf2,WAV_I2S_RX_DMA_BUFSIZE/2,(UINT*)&bw);//写入文件
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
				HAL_I2S_DMAStop(&hi2s2); //关闭dma
error1:
				if (record_begin == 1)
				{
				    wavheadrx->riff.ChunkSize=wavsize+36;		//整个文件的大小-8;
				    wavheadrx->data.ChunkSize=wavsize;		//数据大小
				    res= f_lseek(audiodev.file2,0);						//偏移到文件头.
				    if (res != FR_OK)
				    {
				        app_trace_log("%s,%d,error %d\n",__FUNCTION__,__LINE__,res);
				        goto error1;
				    }
				    res= f_write(audiodev.file2,(const void*)wavheadrx,sizeof(__WaveHeader),&bw);//写入头数据
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
				rec_sta=0;	//关闭中断发送
				play_begin = 0; //关闭播放
				record_begin = 0; //关闭录音
			}
		}
		else
		{
			//未播放时休眠500毫秒
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
        wavwitchrxbuf=0;		//i2sbufx指示标志
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
		wavwitchrxbuf=1;		//i2sbufx指示标志
		xHigherPriorityTaskWorken = pdFALSE;

		vTaskNotifyGiveFromISR(audioplayTaskHandle,&xHigherPriorityTaskWorken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWorken);
    }
}

void HAL_I2SEx_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    UNUSED(hi2s);

    wavtransferend=1;	//i2s传输完成标志
    wavwitchtxbuf=0;		//i2sbufx指示标志
}
void HAL_I2SEx_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    UNUSED(hi2s);

	wavtransferend=1;	//i2s传输完成标志
	wavwitchtxbuf=1;		//i2sbufx指示标志
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

