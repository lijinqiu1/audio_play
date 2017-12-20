/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "gpio.h"
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
extern EventGroupHandle_t xEventGroup;
extern TimerHandle_t xKeyDelayTimer;
static uint16_t key_pressd;
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

#if defined(F429_BIT6)
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IIC_SCLK_Pin
                          |IIC_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, KEY_VOL_UP_Pin|KEY_ASK_Pin|KEY_VOL_DOWN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = IIC_SCLK_Pin
                          |IIC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_WAKE_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_WAKE_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_FUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_FUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PGPin PGPin PGPin */
  GPIO_InitStruct.Pin = KEY_VOL_UP_Pin|KEY_ASK_Pin|KEY_VOL_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, IRQ_PRI_EXIT9_5, IRQ_SUBPRI_EXIT9_5);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, IRQ_PRI_EXIT15_10, IRQ_SUBPRI_EXIT15_10);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  HAL_NVIC_SetPriority(EXTI0_IRQn, IRQ_PRI_EXIT0, IRQ_SUBPRI_EXIT0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#elif defined(F429_ZET6)
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PAIR_BT_GPIO_Port,PAIR_BT_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MODE_BT_GPIO_Port,MODE_BT_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RESET_BT_GPIO_Port,RESET_BT_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IIC_SCLK_GPIO_Port,IIC_SCLK_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IIC_SDA_GPIO_Port,IIC_SDA_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(REF_EN_GPIO_Port,REF_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_CRT_GPIO_Port,USB_CRT_Pin,GPIO_PIN_SET);


    /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = REF_EN_Pin | OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = PAIR_BT_Pin|MODE_BT_Pin|RESET_BT_Pin|IIC_SCLK_Pin
                          |IIC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_WAKE_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_WAKE_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_FUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_FUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PGPin PGPin PGPin */
  GPIO_InitStruct.Pin = KEY_VOL_UP_Pin|KEY_ASK_Pin|KEY_VOL_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Usb connect pin*/
  GPIO_InitStruct.Pin = USB_CRT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(USB_CRT_GPIO_Port,&GPIO_InitStruct);

  GPIO_InitStruct.Pin = OLED_RES_Pin | OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE,&GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, IRQ_PRI_EXIT9_5, IRQ_SUBPRI_EXIT9_5);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, IRQ_PRI_EXIT15_10, IRQ_SUBPRI_EXIT15_10);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  HAL_NVIC_SetPriority(EXTI0_IRQn, IRQ_PRI_EXIT0, IRQ_SUBPRI_EXIT0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

#endif


}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	key_pressd = GPIO_Pin;
	xTimerStartFromISR(xKeyDelayTimer,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void prvKeyDelayCallback(TimerHandle_t xTimer)
{
	uint16_t GPIO_Pin;

	GPIO_Pin = key_pressd;
	if(key_work_status == 1)
	{//播放模式
		if(GPIO_Pin == KEY_VOL_UP_Pin)
		{
			if(HAL_GPIO_ReadPin(KEY_VOL_UP_GPIO_Port,KEY_VOL_UP_Pin))
				xEventGroupSetBits(xEventGroup, EVENTS_VOL_UP_BIT);
		}
		if(GPIO_Pin == KEY_VOL_DOWN_Pin)
		{
			if(HAL_GPIO_ReadPin(KEY_VOL_DOWN_GPIO_Port,KEY_VOL_DOWN_Pin))
				xEventGroupSetBits(xEventGroup, EVENTS_VOL_DOWN_BIT);
		}
		if(GPIO_Pin == KEY_FUN_Pin)
		{

		}
		if(GPIO_Pin == KEY_ASK_Pin)
		{
			if(HAL_GPIO_ReadPin(KEY_ASK_GPIO_Port,KEY_ASK_Pin))
				xEventGroupSetBits(xEventGroup, EVENTS_ASK_BIT);
		}
		if(GPIO_Pin == KEY_WAKE_UP_Pin)
		{
#if defined(F429_BIT6)
			if(HAL_GPIO_ReadPin(KEY_WAKE_UP_GPIO_Port,KEY_WAKE_UP_Pin))
#elif defined(F429_ZET6)
			if(!HAL_GPIO_ReadPin(KEY_WAKE_UP_GPIO_Port,KEY_WAKE_UP_Pin))
#endif
				xEventGroupSetBits(xEventGroup,EVENTS_FUN_STOP_BIT);
		}
	}
	else
	{//待机模式
		if(GPIO_Pin == KEY_VOL_UP_Pin)
		{
			if(HAL_GPIO_ReadPin(KEY_VOL_UP_GPIO_Port,KEY_VOL_UP_Pin))
				xEventGroupSetBits(xEventGroup, EVENTS_PLAY_BIT);
		}
		if(GPIO_Pin == KEY_VOL_DOWN_Pin)
		{
			if(HAL_GPIO_ReadPin(KEY_VOL_DOWN_GPIO_Port,KEY_VOL_DOWN_Pin))
				xEventGroupSetBits(xEventGroup, EVENTS_RECORD_BIT);
		}
		if(GPIO_Pin == KEY_FUN_Pin)
		{
			if(HAL_GPIO_ReadPin(KEY_FUN_GPIO_Port,KEY_FUN_Pin))
				xEventGroupSetBits(xEventGroup, EVENTS_FUN_BLE_CHANGE_BIT);
		}
		if(GPIO_Pin == KEY_ASK_Pin)
		{
			if(HAL_GPIO_ReadPin(KEY_ASK_GPIO_Port,KEY_ASK_Pin))
				xEventGroupSetBits(xEventGroup, EVENTS_PLAY_AND_RECORD_BIT);
		}
		if(GPIO_Pin== KEY_WAKE_UP_Pin)
		{
			if(!HAL_GPIO_ReadPin(KEY_WAKE_UP_GPIO_Port,KEY_WAKE_UP_Pin))
				xEventGroupSetBits(xEventGroup, EVENTS_FUN_USB_BIT);
		}
	}
}

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
