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
//���������еİ����¼�
//
//*******************************************************************************

void Events_Process_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1000);
	}
}

