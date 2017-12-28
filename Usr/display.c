#include "main.h"
#include "freertos.h"
#include "display.h"
#include "oled.h"
#include "bmp.h"
#include "rtc.h"
#include "cmsis_os.h"


osThreadId displayprocessHandle;

void Display_Time(void)
{
	uint8_t time_buffer[6];
	RTC_TimeTypeDef tim;
	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
	sprintf((char*)time_buffer,"%02d:%02d",tim.Hours,tim.Minutes);
	OLED_ShowString(DISPLAY_TIME_X,DISPLAY_TIME_Y,time_buffer,12);
}

void Display_Process_Task(void const * argument)
{
    OLED_Init();
    OLED_Clear();
    while(1)
    {
		Display_Time();
		osDelay(1000);
    }
}


