#include <string.h>
#include "main.h"
#include "events.h"
#include "audio.h"
#include "ff.h"
#include "freertos.h"
#include "rtc.h"
#include "rtc.h"

extern EventGroupHandle_t xEventGroup;

//*******************************************************************************
//负责处理所有的按键事件
//
//*******************************************************************************

void Events_Process_Task(void const * argument)
{
	EventBits_t xEventGroupValue;
	RTC_TimeTypeDef tim;
	char fname[30];
	char buffer[30];
	const EventBits_t xBitsToWaitFor = (EVENTS_FUN_STOP_BIT |
	                                     EVENTS_PLAY_BIT |
	                                     EVENTS_RECORD_BIT |
	                                     EVENTS_PLAY_AND_RECORD_BIT);

	const EventBits_t uxAllSyncBits = ( EVENTS_VOL_UP_BIT |
										EVENTS_VOL_DOWN_BIT |
										EVENTS_FUN_STOP_BIT |
										EVENTS_PLAY_BIT |
										EVENTS_RECORD_BIT |
										EVENTS_PLAY_AND_RECORD_BIT |
										EVENTS_PLAY_END_BIT |
										EVENTS_RECORD_END_BIT |
										EVENTS_PLAY_AND_RECORD_END_BIT |
										EVENTS_ASK_BIT |
										EVENTS_FUN_BLE_OPEN_BIT|
										EVENTS_FUN_BLE_CLOSE_BIT);
	for(;;)
	{
		xEventGroupValue = xEventGroupWaitBits(/* The event group to read. */
													   xEventGroup,
													   /* Bits to test. */
													   uxAllSyncBits,
													   /* Clear bits on exit if the
																								  unblock condition is met. */
													   pdTRUE,
													   /* Don't wait for all bits. This
																								  parameter is set to pdTRUE for the
																								  second execution. */
													   pdFALSE,
													   /* Don't time out. */
													   portMAX_DELAY);
		//等待事件同步
		xEventGroupSync(xEventGroup,
						uxAllSyncBits,
						uxAllSyncBits,
						portMAX_DELAY);
		if((xEventGroupValue&EVENTS_VOL_UP_BIT)!=0)
		{//音量+
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	VOL_UP\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_VOL_DOWN_BIT)!=0)
		{//音量-
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	VOL_Donw\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_ASK_BIT)!=0)
		{//报告
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	REPORT\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_FUN_BLE_OPEN_BIT)!=0)
		{//蓝牙打开
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	BLUETOOTH OPEN\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_FUN_BLE_CLOSE_BIT)!=0)
		{//蓝牙关闭
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	BLUETOOTH CLOSE\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_FUN_STOP_BIT)!=0)
		{//停止播放
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	STOP\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_PLAY_BIT)!=0)
		{//报告
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	REPORT\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_ASK_BIT)!=0)
		{//报告
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	REPORT\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_ASK_BIT)!=0)
		{//报告
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	REPORT\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_ASK_BIT)!=0)
		{//报告
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	REPORT\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
		if((xEventGroupValue&EVENTS_ASK_BIT)!=0)
		{//报告
			HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
			sprintf(buffer,"%d:%d%:%d:%d	REPORT\n",tim.Hours,tim.Minutes,tim.Seconds,tim.SubSeconds);
		}
	}
}

