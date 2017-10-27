#include <string.h>
#include "main.h"
#include "log.h"
#include "audio.h"
#include "ff.h"
#include "freertos.h"
#include "rtc.h"
#include "rtc.h"

//队列
QueueHandle_t xQueueLog;

osThreadId logrecordHandle;

extern EventGroupHandle_t xEventGroup;
extern SemaphoreHandle_t xSdioMutex;
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
	BaseType_t xStatus;
	char fname[30];
	char queuebuf[QUEUE_LOG_ITEM_SIZE];
	UINT bw;
	DWORD file_long = 0;

	//打开log文件夹，如果没有创建
	while(f_opendir(&recdir,"0:/LOG"))
	{
		res = f_mkdir("0:/LOG");
		APP_ERROR_CHECK(res);
	}
	f_closedir(&recdir);

	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);
	sprintf(fname,"%sLOG-%d-%d-%d.txt",LOG_PATH,dat.Year+2000,dat.Month,dat.Date);
	res = f_open(&log_file,fname,FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
	APP_ERROR_CHECK(res);
	res = f_read(&log_file,queuebuf,QUEUE_LOG_ITEM_SIZE,&bw);
	APP_ERROR_CHECK(res);
	while(bw == QUEUE_LOG_ITEM_SIZE)
	{
		res = f_read(&log_file,queuebuf,QUEUE_LOG_ITEM_SIZE,&bw);
		APP_ERROR_CHECK(res);
		file_long += QUEUE_LOG_ITEM_SIZE;
	}
	file_long += bw;
	app_trace_log("%s file_length %d\n",__FUNCTION__,file_long);
	res = f_lseek(&log_file,file_long);
	APP_ERROR_CHECK(res);
	for(;;)
	{
		xStatus = xQueueReceive(xQueueLog,queuebuf,portMAX_DELAY);
		if (xStatus == pdPASS)
		{
			xSemaphoreTake(xSdioMutex,portMAX_DELAY);
			f_puts((const TCHAR*) queuebuf,&log_file);
			f_sync(&log_file);
			xSemaphoreGive(xSdioMutex);
		}
	}
}
