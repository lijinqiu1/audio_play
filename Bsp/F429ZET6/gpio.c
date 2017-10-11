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
  HAL_GPIO_WritePin(GPIOB, PAIR_BT_PB1_Pin|MODE_BT_PB10_Pin|RESET_BT_PB11_Pin|IIC_SCLK_PB8_Pin
                          |IIC_SDA_PB9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, KEY_VOL_UP_PG9_Pin|KEY_ASK_PG10_Pin|KEY_VOL_DOWN_PG11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(REF_EN_PC4_GPIO_Port,REF_EN_PC4_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = REF_EN_PC4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REF_EN_PC4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = PAIR_BT_PB1_Pin|MODE_BT_PB10_Pin|RESET_BT_PB11_Pin|IIC_SCLK_PB8_Pin
                          |IIC_SDA_PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_WAKE_UP_PA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_WAKE_UP_PA0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_FUN_PG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_FUN_PG12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PGPin PGPin PGPin */
  GPIO_InitStruct.Pin = KEY_VOL_UP_PG9_Pin|KEY_ASK_PG10_Pin|KEY_VOL_DOWN_PG11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(key_work_status == 1)
	{//播放模式
		if(GPIO_Pin == KEY_VOL_UP_PG9_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_VOL_UP_BIT, &xHigherPriorityTaskWoken);
		}
		if(GPIO_Pin == KEY_VOL_DOWN_PG11_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_VOL_DOWN_BIT, &xHigherPriorityTaskWoken);
		}
		if(GPIO_Pin == KEY_FUN_PG12_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_VOL_UP_BIT, &xHigherPriorityTaskWoken);
		}
		if(GPIO_Pin == KEY_ASK_PG10_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_ASK_BIT, &xHigherPriorityTaskWoken);
		}
		if(GPIO_Pin== KEY_WAKE_UP_PA0_Pin)
		{

		}
	}
	else
	{//待机模式
		if(GPIO_Pin == KEY_VOL_UP_PG9_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_PLAY_BIT, &xHigherPriorityTaskWoken);
		}
		if(GPIO_Pin == KEY_VOL_DOWN_PG11_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_RECORD_BIT, &xHigherPriorityTaskWoken);
		}
		if(GPIO_Pin == KEY_FUN_PG12_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_FUN_BLE_CHANGE_BIT, &xHigherPriorityTaskWoken);
		}
		if(GPIO_Pin == KEY_ASK_PG10_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_PLAY_AND_RECORD_BIT, &xHigherPriorityTaskWoken);
		}
		if(GPIO_Pin== KEY_WAKE_UP_PA0_Pin)
		{
			xEventGroupSetBitsFromISR(xEventGroup, EVENTS_FUN_BLE_PAIR_BIT, &xHigherPriorityTaskWoken);
		}

	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
