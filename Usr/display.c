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
#include "audio.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_msc.h"
#include "usbd_storage_if.h"
#include "usbd_composite.h"

extern EventGroupHandle_t xEventGroup;
osThreadId displayprocessHandle;

static void DisPlay_Start(void)
{
    OLED_ShowCHinese(16, 2, 0, (uint8_t (*)[32])HzPlay);
    OLED_ShowCHinese(32, 2, 1, (uint8_t (*)[32])HzPlay);
}

static void DisPlay_Pause(void)
{
    OLED_ShowCHinese(16, 2, 0, (uint8_t (*)[32])HzPause);
    OLED_ShowCHinese(32, 2, 1, (uint8_t (*)[32])HzPause);
}
//static void DisPlay_Stop(void)
//{
//    OLED_ShowCHinese(16, 2, 0, (uint8_t (*)[32])HzSTOP);
//    OLED_ShowCHinese(32, 2, 1, (uint8_t (*)[32])HzSTOP);
//}

static void DisPlay_Task(void)
{
    if (cur_task_index < 10)
    {
        OLED_ShowCHinese(8, 2, 0, (uint8_t (*)[32])HzTask);
        OLED_ShowCHinese(24, 2, 1, (uint8_t (*)[32])HzTask);
        OLED_ShowNum(40, 2, cur_task_index, 1, 16);
    }
    else if(cur_task_index < 100)
    {
        OLED_ShowCHinese(8, 2, 0, (uint8_t (*)[32])HzTask);
        OLED_ShowCHinese(24, 2, 1, (uint8_t (*)[32])HzTask);
        OLED_ShowNum(40, 2, cur_task_index, 2, 16);
    }
    else
    {
        OLED_ShowCHinese(8, 2, 0, (uint8_t (*)[32])HzTask);
        OLED_ShowCHinese(24, 2, 1, (uint8_t (*)[32])HzTask);
        OLED_ShowNum(40, 2, cur_task_index, 3, 16);
    }
    OLED_DrawBMP(56, 2, 64, 4, (unsigned char *)direction);
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
    static uint32_t play_seconds_count = 0;
    uint8_t minutes = 0;
    uint8_t seconds = 0;
    static uint8_t last_work_status = KEY_WORK_STATUS_PLAY;
    static RTC_TimeTypeDef last_tim;
    if(last_work_status != key_work_status)
    {
        last_work_status = key_work_status;
        if (key_work_status == KEY_WORK_STATUS_PLAY)
        {
            memcpy((char *)&last_tim,(char *)&tim,sizeof(RTC_TimeTypeDef));
            OLED_Clear();
        }
        else
        {
            play_seconds_count = 0;
            task_play_status = TASK_PLAY_STATUS_PLAYING;
        }
    }
    if (key_work_status == KEY_WORK_STATUS_READY)
    {
        DisPlay_Task();
    }
    else if (key_work_status == KEY_WORK_STATUS_PLAY)
    {
        if(task_play_status == TASK_PLAY_STATUS_PLAYING)
        {
            play_seconds_count++;
            seconds = play_seconds_count % 60;
            minutes = play_seconds_count / 60;
            DisPlay_Start();
            sprintf((char *)time_buffer,"%02d:%02d",minutes,seconds);
	        OLED_ShowString(DISPLAY_WORK_STATUS_X,DISPLAY_WORK_STATUS_Y,time_buffer,12);
        }
        else
        {
            DisPlay_Pause();
        }

        /*
        DisPlay_Start();
		if (last_tim.Hours == tim.Hours)
		{
            minutes = ((tim.Minutes * 60 + tim.Seconds) - \
                (last_tim.Minutes * 60 + last_tim.Seconds)) / 60;
            seconds = ((tim.Minutes * 60 + tim.Seconds) - \
                (last_tim.Minutes * 60 + last_tim.Seconds)) % 60;
        }
        else if(last_tim.Hours < tim.Hours)
        {
            minutes = ((tim.Hours * 3600 + tim.Minutes * 60 + tim.Seconds) - \
                (last_tim.Hours * 3600 + last_tim.Minutes * 60 + last_tim.Seconds)) / 60;
            seconds = ((tim.Hours * 3600 + tim.Minutes * 60 + tim.Seconds) - \
                (last_tim.Hours * 3600 + last_tim.Minutes * 60 + last_tim.Seconds)) % 60;
        }
        else if(last_tim.Hours > tim.Hours)
        {
            minutes = (((tim.Hours + 24) * 3600 + tim.Minutes * 60 + tim.Seconds) - \
                (last_tim.Hours * 3600 + last_tim.Minutes * 60 + last_tim.Seconds)) / 60;
            seconds = (((tim.Hours + 24) * 3600 + tim.Minutes * 60 + tim.Seconds) - \
                (last_tim.Hours * 3600 + last_tim.Minutes * 60 + last_tim.Seconds)) % 60;
        }
        sprintf((char *)time_buffer,"%02d:%02d",minutes,seconds);
	    OLED_ShowString(DISPLAY_WORK_STATUS_X,DISPLAY_WORK_STATUS_Y,time_buffer,12);
	    */
    }
}

static void Display_Battery_Value(uint16_t *battery_value)
{
    uint16_t value = battery_value[0] * 1.0 /battery_value[1] * 4.2;
    static uint8_t index = 0;
    if(HAL_GPIO_ReadPin(VBUS_DET_GPIO_Port, VBUS_DET_Pin) == GPIO_PIN_RESET)
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
	RTC_DateTypeDef dat;
	RTC_TimeTypeDef tim;
	GPIO_InitTypeDef GPIO_InitStruct;
	EventBits_t xEventGroupValue;
	const EventBits_t xBitsToWaitFor = (EVENTS_KEY_UP_BIT|EVENTS_KEY_DOWN_BIT);
    while(1)
    {
        if(HAL_GPIO_ReadPin(VBUS_DET_GPIO_Port, VBUS_DET_Pin))
        {
            usb_connect_status = USB_CONNECT_STATUS_CONNECTED;
        }
        else
        {
            usb_connect_status = USB_CONNECT_STATUS_DISCONNECTED;
			if(Device_Status == Device_USB_CONNECTED)
			{
				Device_Status = Device_Work_Normal;
				USBD_DeInit(&hUsbDeviceFS);
				  GPIO_InitStruct.Pin = GPIO_PIN_12;
				  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
				  GPIO_InitStruct.Pull = GPIO_NOPULL;
				  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
			}
        }
        //¶ÂÈûÄ£Ê½
        xEventGroupValue = xEventGroupWaitBits(/* The event group to read. */
                                               xEventGroup,
                                               /* Bits to test. */
                                               xBitsToWaitFor,
                                               /* Clear bits on exit if the
                                                unblock condition is met. */
                                               pdTRUE,
                                               /* Don't wait for all bits. This
                                                parameter is set to pdTRUE for the
                                                second execution. */
                                               pdFALSE,
                                               /* Don't time out. */
                                               1000);
        if((xEventGroupValue&EVENTS_KEY_UP_BIT)!=0)
        {
            cur_task_index --;
            if (cur_task_index == 0)
            {
                cur_task_index = task_count;
            }
        }

        if((xEventGroupValue&EVENTS_KEY_DOWN_BIT)!=0)
        {
            cur_task_index ++;
            if (cur_task_index > task_count)
            {
                cur_task_index = 1;
            }
        }

    	HAL_RTC_GetTime(&hrtc,&tim,RTC_FORMAT_BIN);
    	HAL_RTC_GetDate(&hrtc,&dat,RTC_FORMAT_BIN);
		Display_Time(tim);
		if (Device_Status == Device_Work_Normal)
			Display_work_status(tim);
		
        Display_Battery_Value(battery_value);
        //app_trace_log("value = %f\n",(battery_value[0]*1.0/battery_value[1])*4.2);
    }
}


