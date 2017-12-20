#include "main.h"
#include "freertos.h"
#include "display.h"
#include "oled.h"
#include "bmp.h"
#include "cmsis_os.h"


osThreadId displayprocessHandle;


void Display_Process_Task(void const * argument)
{
    OLED_Init();
    OLED_Clear();

    OLED_ShowString(0, 0, "begin");

    while(1)
    {
		OLED_Clear();
	 
		OLED_ShowCHinese(8,0,0);//中
		OLED_ShowCHinese(26,0,1);//景
		OLED_ShowCHinese(44,0,2);//园
		OLED_ShowString(16,2,"0.66");
		OLED_ShowString(0,4,"OLEDTEST");
        osDelay(8000);
		OLED_Clear();
		OLED_DrawBMP(0,0,64,6,(unsigned char *)BMP1);  //图片显示(图片显示慎用，生成的字表较大，会占用较多空间，FLASH空间8K以下慎用)
		osDelay(8000);
    }
}


