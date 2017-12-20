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
	 
		OLED_ShowCHinese(8,0,0);//��
		OLED_ShowCHinese(26,0,1);//��
		OLED_ShowCHinese(44,0,2);//԰
		OLED_ShowString(16,2,"0.66");
		OLED_ShowString(0,4,"OLEDTEST");
        osDelay(8000);
		OLED_Clear();
		OLED_DrawBMP(0,0,64,6,(unsigned char *)BMP1);  //ͼƬ��ʾ(ͼƬ��ʾ���ã����ɵ��ֱ�ϴ󣬻�ռ�ý϶�ռ䣬FLASH�ռ�8K��������)
		osDelay(8000);
    }
}


