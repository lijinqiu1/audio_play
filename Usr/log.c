#include <string.h>
#include "main.h"
#include "log.h"
#include "audio.h"
#include "ff.h"
#include "freertos.h"
#include "rtc.h"
#include "rtc.h"

<<<<<<< HEAD
//队列
QueueHandle_t xQueueLog;

=======

extern EventGroupHandle_t xEventGroup;
>>>>>>> 141c260505ccf0862c751e142a69b947af2288f0
//*******************************************************************************
//事件记录线程
//负责写入文件log
//*******************************************************************************
void Log_Record_Task(void const * argument)
{
	FRESULT res;
	FIL log_file;
	DIR recdir;
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
	BaseType_t xStatus;
	char fname[30];
	char queuebuf[QUEUE_LOG_ITEM_SIZE];

	//打开log文件夹，如果没有创建
	while(f_opendir(&recdir,"0:/LOG"))
	{
		res = f_mkdir("0:/LOG");
		APP_ERROR_CHECK(res);
	}

	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);
<<<<<<< HEAD
	sprintf(fname,"LOG-%d-%d-%d.txt",dat.Year,dat.Month,dat.Date);
	res = f_open(&log_file,fname,FA_OPEN_ALWAYS|FA_WRITE);
=======
	res = f_open(&log_file,"0:/LOG/x.txt",FA_OPEN_ALWAYS|FA_WRITE);
>>>>>>> 141c260505ccf0862c751e142a69b947af2288f0
	APP_ERROR_CHECK(res);

	for(;;)
	{
		xStatus = xQueueReceive(xQueueLog,queuebuf,portMAX_DELAY);
		if (xStatus == pdPASS)
		{
			res = f_puts((const TCHAR*) queuebuf,&log_file);
			APP_ERROR_CHECK(res);
		}
	}
}
