#include "main.h"
#include "freertos.h"
#include "display.h"
#include "bmp.h"
#include "rtc.h"
#include "adc.h"
#include "bmp.h"
#include "oled.h"
#include "oledhzfont.h"
#include "cmsis_os.h"


osThreadId displayprocessHandle;

static void DisPlay_Start(void)
{
    OLED_ShowCHinese(16, 2, 0, (uint8_t (*)[32])HzSTART);
    OLED_ShowCHinese(32, 2, 1, (uint8_t (*)[32])HzSTART);
}

static void DisPlay_Stop(void)
{
    OLED_ShowCHinese(16, 2, 0, (uint8_t (*)[32])HzSTOP);
    OLED_ShowCHinese(32, 2, 1, (uint8_t (*)[32])HzSTOP);
}


static void Display_Time(RTC_TimeTypeDef tim)
{
	uint8_t time_buffer[6];
	sprintf((char*)time_buffer,"%02d:%02d",tim.Hours,tim.Minutes);
	OLED_ShowString(DISPLAY_TIME_X,DISPLAY_TIME_Y,time_buffer,12);
    //app_trace_log("%s,%d-%d-%d\n",time_buffer,tim.Hours,tim.Minutes,tim.Seconds);
}

static void Display_work_status(RTC_TimeTypeDef tim)
{
	uint8_t time_buffer[6];
    static uint8_t last_work_status = KEY_WORK_STATUS_PLAY;
    uint8_t minutes = 0;
    uint8_t seconds = 0;
    static RTC_TimeTypeDef last_tim;
    if(last_work_status != key_work_status)
    {
        last_work_status = key_work_status;
        if (key_work_status == KEY_WORK_STATUS_READY)
        {
            sprintf((char *)time_buffer,"00:00");
            DisPlay_Stop();
	        OLED_ShowString(DISPLAY_WORK_STATUS_X,DISPLAY_WORK_STATUS_Y,time_buffer,12);
        }
        else
        {
            DisPlay_Start();
            memcpy((char *)&last_tim,(char *)&tim,sizeof(RTC_TimeTypeDef));
        }
    }
    else if (key_work_status == KEY_WORK_STATUS_PLAY)
    {
		if (tim.Hours >= last_tim.Hours)
		{
			minutes = (tim.Hours - last_tim.Hours) * 60 ;
		}
		else
		{
			minutes = (last_tim.Hours - tim.Hours) * 60;
		}

        if (tim.Minutes >= last_tim.Minutes)
        {
            minutes += tim.Minutes - last_tim.Minutes;
        }
        else
        {
            minutes += last_tim.Minutes - tim.Minutes;
        }
        
		if (tim.Seconds >= last_tim.Seconds)
		{
			seconds = tim.Seconds - last_tim.Seconds;
		}
		else
		{
			seconds = 60 + tim.Seconds - last_tim.Seconds;
		}
        sprintf((char *)time_buffer,"%02d:%02d",minutes,seconds);
	    OLED_ShowString(DISPLAY_WORK_STATUS_X,DISPLAY_WORK_STATUS_Y,time_buffer,12);
    }
}

static void Display_Battery_Value(uint16_t *battery_value)
{
    uint16_t value = battery_value[0] * 1.0 /battery_value[1] * 4.2;
    static uint8_t index = 0;
    if(usb_connect_status == USB_CONNECT_STATUS_DISCONNECTED)
    {
        if(value > 3.8)
        {
            OLED_DrawBMP(40,0,64,1,(unsigned char *)BATTERY_FULL);
        }
        else if(value > 3.7)
        {
            OLED_DrawBMP(40,0,64,1,(unsigned char *)BATTERY_2);
        }
        else if(value > 3.6)
        {
            OLED_DrawBMP(40,0,64,1,(unsigned char *)BATTERY_1);
        }
        else if(value > 3.4)
        {
            OLED_DrawBMP(40,0,64,1,(unsigned char *)BATTERY_0);
        }
        index = 0;
    }
    else
    {
        switch(index)
        {
        case 0:
            OLED_DrawBMP(40,0,64,1,(unsigned char *)BATTERY_0);
            index ++;
        break;
        case 1:
            OLED_DrawBMP(40,0,64,1,(unsigned char *)BATTERY_1);
            index ++;
        break;
        case 2:
            OLED_DrawBMP(40,0,64,1,(unsigned char *)BATTERY_2);
            index ++;
        break;
        case 3:
            OLED_DrawBMP(40,0,64,1,(unsigned char *)BATTERY_FULL);
            index = 0;
        break;
        }
    }
}


void Display_Process_Task(void const * argument)
{
    uint16_t battery_value[2] = {4096,4096};
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
    OLED_Init();
    OLED_Clear();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)battery_value, 2);
    app_trace_log("Display_Process_Task begin\n");
    while(1)
    {
        if(HAL_GPIO_ReadPin(VBUS_DET_GPIO_Port, VBUS_DET_Pin))
        {
            usb_connect_status = USB_CONNECT_STATUS_CONNECTED;
        }
        else
        {
            usb_connect_status = USB_CONNECT_STATUS_DISCONNECTED;
        }
    	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
    	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);
		Display_Time(tim);
        Display_work_status(tim);
        Display_Battery_Value(battery_value);
        //app_trace_log("value = %f\n",(battery_value[0]*1.0/battery_value[1])*4.2);
		osDelay(1000);
    }
}


