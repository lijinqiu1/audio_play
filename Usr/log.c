#include <string.h>
#include "main.h"
#include "log.h"
#include "wav.h"
#include "ff.h"
#include "freertos.h"
#include "rtc.h"
#include "rtc.h"


extern EventGroupHandle_t xEventGroup;
//*******************************************************************************
//事件记录线程
//*******************************************************************************
void Log_Record_Task(void const * argument)
{
	FRESULT res;
	FIL log_file;
	EventBits_t xEventGroupValue;
	DIR recdir;
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;

	const EventBits_t uxAllSyncBits = ( EVENTS_VOL_UP_BIT |
										EVENTS_VOL_DOWN_BIT |
										EVENTS_FUN_STOP_BIT |
										EVENTS_PLAY_BIT |
										EVENTS_RECORD_BIT |
										EVENTS_PLAY_AND_RECORD_BIT |
										EVENTS_ASK_BIT |
										EVENTS_FUN_BLE_BIT);

	//打开log文件夹，如果没有创建
	while(f_opendir(&recdir,"0:/LOG"))
	{
		res = f_mkdir("0:/LOG");
		APP_ERROR_CHECK(res);
	}

	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);
	res = f_open(&log_file,"0:/LOG/x.txt",FA_OPEN_ALWAYS|FA_WRITE);
	APP_ERROR_CHECK(res);

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
		                                       0);
		//等待事件同步
		xEventGroupSync(xEventGroup,
						uxAllSyncBits,
						uxAllSyncBits,
						portMAX_DELAY);
	}
}
