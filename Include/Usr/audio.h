#ifndef __AUDIO_H__
#define __AUDIO_H__
#include "main.h"
#include "ff.h"
#include "cmsis_os.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//WAV �������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/6/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//V1.0 ˵��
//1,֧��16λ/24λWAV�ļ�����
//2,��߿���֧�ֵ�192K/24bit��WAV��ʽ.
//////////////////////////////////////////////////////////////////////////////////


#define WAV_I2S_TX_DMA_BUFSIZE    12288		//����WAV TX DMA �����С(����192Kbps@24bit��ʱ��,��Ҫ����8192��Ų��Ῠ)
#define WAV_I2S_RX_DMA_BUFSIZE    WAV_I2S_TX_DMA_BUFSIZE

//RIFF��
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;����̶�Ϊ"RIFF",��0X46464952
    uint32_t ChunkSize ;		   	//���ϴ�С;�ļ��ܴ�С-8
    uint32_t Format;	   			//��ʽ;WAVE,��0X45564157
}ChunkRIFF ;
//fmt��
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;����̶�Ϊ"fmt ",��0X20746D66
    uint32_t ChunkSize ;		   	//�Ӽ��ϴ�С(������ID��Size);����Ϊ:20.
    uint16_t AudioFormat;	  	//��Ƶ��ʽ;0X01,��ʾ����PCM;0X11��ʾIMA ADPCM
	uint16_t NumOfChannels;		//ͨ������;1,��ʾ������;2,��ʾ˫����;
	uint32_t SampleRate;			//������;0X1F40,��ʾ8Khz
	uint32_t ByteRate;			//�ֽ�����;
	uint16_t BlockAlign;			//�����(�ֽ�);
	uint16_t BitsPerSample;		//�����������ݴ�С;4λADPCM,����Ϊ4
//	u16 ByteExtraData;		//���ӵ������ֽ�;2��; ����PCM,û���������
}ChunkFMT;
//fact��
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;����̶�Ϊ"fact",��0X74636166;
    uint32_t ChunkSize ;		   	//�Ӽ��ϴ�С(������ID��Size);����Ϊ:4.
    uint32_t NumOfSamples;	  	//����������;
}ChunkFACT;
//LIST��
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;����̶�Ϊ"LIST",��0X74636166;
    uint32_t ChunkSize ;		   	//�Ӽ��ϴ�С(������ID��Size);����Ϊ:4.
}ChunkLIST;

//data��
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;����̶�Ϊ"data",��0X5453494C
    uint32_t ChunkSize ;		   	//�Ӽ��ϴ�С(������ID��Size)
}ChunkDATA;

//wavͷ
typedef __packed struct
{
	ChunkRIFF riff;	//riff��
	ChunkFMT fmt;  	//fmt��
//	ChunkFACT fact;	//fact�� ����PCM,û������ṹ��
	ChunkDATA data;	//data��
}__WaveHeader;

//wav ���ſ��ƽṹ��
typedef __packed struct
{
    uint16_t audioformat;			//��Ƶ��ʽ;0X01,��ʾ����PCM;0X11��ʾIMA ADPCM
	uint16_t nchannels;				//ͨ������;1,��ʾ������;2,��ʾ˫����;
	uint16_t blockalign;				//�����(�ֽ�);
	uint32_t datasize;				//WAV���ݴ�С

    uint32_t totsec ;				//���׸�ʱ��,��λ:��
    uint32_t cursec ;				//��ǰ����ʱ��

    uint32_t bitrate;	   			//������(λ��)
	uint32_t samplerate;				//������
	uint16_t bps;					//λ��,����16bit,24bit,32bit

	uint32_t datastart;				//����֡��ʼ��λ��(���ļ������ƫ��)
}__wavctrl;

//���ֲ��ſ�����
typedef __packed struct
{
	//2��I2S�����BUF
	uint8_t *i2sbuf1;
	uint8_t *i2sbuf2;
	uint8_t *tbuf;				//��ʱ����,����24bit�����ʱ����Ҫ�õ�
	FIL *file1;				//��Ƶ�ļ�ָ��
	FIL *file2;				//��Ƶ�ļ�ָ��

	uint8_t status;				//bit0:0,��ͣ����;1,��������
							//bit1:0,��������;1,��������
}__audiodev;

void AudioPlay_Task(void const * argument);
#if defined(PLAY_WITH_LIST)
void AudioPlay_With_List_Task(void const * argument);
#elif defined(PLAY_WITH_RNG)
void AudioPlay_Task(void const * argument);
#endif
void AudioController_Task(void const *argument);
#endif

















