#include <string.h>
#include "main.h"
#include "events.h"
#include "audio.h"
#include "ff.h"
#include "freertos.h"
#include "rtc.h"

extern EventGroupHandle_t xEventGroup;
osThreadId eventsprocessHandle;

//*******************************************************************************
//负责处理所有的按键事件
//
//*******************************************************************************

void Events_Process_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1000);
	}
}

