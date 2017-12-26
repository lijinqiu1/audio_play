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
	sprintf((char*)time_buffer,"%2d:%2d",tim.Hours,tim.Seconds);
	OLED_ShowString(DISPLAY_TIME_X,DISPLAY_TIME_Y,time_buffer,12);
}

void Display_Process_Task(void const * argument)
{
    OLED_Init();
    OLED_Clear();

    OLED_ShowString(0, 0, "begin",SIZE);

    while(1)
    {
		OLED_Clear();

		OLED_ShowCHinese(8,0,0);//中
		OLED_ShowCHinese(26,0,1);//景
		OLED_ShowCHinese(44,0,2);//园
		OLED_ShowString(16,2,"0.66",SIZE);
		OLED_ShowString(0,4,"OLEDTEST",SIZE);
        osDelay(8000);
		OLED_Clear();
		OLED_DrawBMP(0,0,64,6,(unsigned char *)BMP1);  //图片显示(图片显示慎用，生成的字表较大，会占用较多空间，FLASH空间8K以下慎用)
		osDelay(8000);
    }
}


