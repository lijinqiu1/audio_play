#ifndef __AUDIO_H__
#define __AUDIO_H__
#include "main.h"
#include "ff.h"
#include "cmsis_os.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//WAV 解码代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/29
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//V1.0 说明
//1,支持16位/24位WAV文件播放
//2,最高可以支持到192K/24bit的WAV格式.
//////////////////////////////////////////////////////////////////////////////////


#define WAV_I2S_TX_DMA_BUFSIZE    16384		//定义WAV TX DMA 数组大小(播放192Kbps@24bit的时候,需要设置8192大才不会卡)
#define WAV_I2S_RX_DMA_BUFSIZE    15360

//RIFF块
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;这里固定为"RIFF",即0X46464952
    uint32_t ChunkSize ;		   	//集合大小;文件总大小-8
    uint32_t Format;	   			//格式;WAVE,即0X45564157
}ChunkRIFF ;
//fmt块
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;这里固定为"fmt ",即0X20746D66
    uint32_t ChunkSize ;		   	//子集合大小(不包括ID和Size);这里为:20.
    uint16_t AudioFormat;	  	//音频格式;0X01,表示线性PCM;0X11表示IMA ADPCM
	uint16_t NumOfChannels;		//通道数量;1,表示单声道;2,表示双声道;
	uint32_t SampleRate;			//采样率;0X1F40,表示8Khz
	uint32_t ByteRate;			//字节速率;
	uint16_t BlockAlign;			//块对齐(字节);
	uint16_t BitsPerSample;		//单个采样数据大小;4位ADPCM,设置为4
//	u16 ByteExtraData;		//附加的数据字节;2个; 线性PCM,没有这个参数
}ChunkFMT;
//fact块
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;这里固定为"fact",即0X74636166;
    uint32_t ChunkSize ;		   	//子集合大小(不包括ID和Size);这里为:4.
    uint32_t NumOfSamples;	  	//采样的数量;
}ChunkFACT;
//LIST块
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;这里固定为"LIST",即0X74636166;
    uint32_t ChunkSize ;		   	//子集合大小(不包括ID和Size);这里为:4.
}ChunkLIST;

//data块
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;这里固定为"data",即0X5453494C
    uint32_t ChunkSize ;		   	//子集合大小(不包括ID和Size)
}ChunkDATA;

//wav头
typedef __packed struct
{
	ChunkRIFF riff;	//riff块
	ChunkFMT fmt;  	//fmt块
//	ChunkFACT fact;	//fact块 线性PCM,没有这个结构体
	ChunkDATA data;	//data块
}__WaveHeader;

//wav 播放控制结构体
typedef __packed struct
{
    uint16_t audioformat;			//音频格式;0X01,表示线性PCM;0X11表示IMA ADPCM
	uint16_t nchannels;				//通道数量;1,表示单声道;2,表示双声道;
	uint16_t blockalign;				//块对齐(字节);
	uint32_t datasize;				//WAV数据大小

    uint32_t totsec ;				//整首歌时长,单位:秒
    uint32_t cursec ;				//当前播放时长

    uint32_t bitrate;	   			//比特率(位速)
	uint32_t samplerate;				//采样率
	uint16_t bps;					//位数,比如16bit,24bit,32bit

	uint32_t datastart;				//数据帧开始的位置(在文件里面的偏移)
}__wavctrl;

//音乐播放控制器
typedef __packed struct
{
	//2个I2S解码的BUF
	uint8_t      *i2sbuf1;
	uint8_t      *i2sbuf2;
	uint8_t      *tbuf;				    //零时数组,仅在24bit解码的时候需要用到
    __wavctrl    wavctrl;               //wav文件描述播放文件使用
    __WaveHeader wavHeader;             //wav文件头录音文件使用
	uint8_t      status;				//bit0:0,暂停播放;1,继续播放
							            //bit1:0,结束播放;1,开启播放
	uint32_t     file_count;
    uint32_t     file_bw;
    uint32_t     wavsize;               //录音文件大小
    uint8_t      file_list[100];
    uint8_t      file_index;            //当前文件播放序列
}__audiodev;

extern uint8_t task_count;
extern uint8_t cur_task_index;
void AudioPlay_Task(void const * argument);
uint8_t Get_task_Count(void);
#if defined(PLAY_WITH_LIST)
#if defined(IIS_DMA_A) || defined(IIS_DMA_B)
void AudioPlay_With_List_Task(void const * argument);
#else
void AudioPlay_With_List_Tx_Task(void const * argument);
void AudioPlay_With_List_Rx_Task(void const * argument);
#endif
#elif defined(PLAY_WITH_RNG)
void AudioPlay_Task(void const * argument);
#endif
void AudioController_Task(void const *argument);
#endif

















