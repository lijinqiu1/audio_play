/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#if defined(F429_BIT6)
#define KEY_Pin GPIO_PIN_8
#define KEY_GPIO_Port GPIOI
#define FLASH_CS_Pin GPIO_PIN_3
#define FLASH_CS_GPIO_Port GPIOG
#define FLASH_SCK_Pin GPIO_PIN_3
#define FLASH_SCK_GPIO_Port GPIOB
#define FLASH_MISO_Pin GPIO_PIN_4
#define FLASH_MISO_GPIO_Port GPIOB
#define FLASH_MOSI_Pin GPIO_PIN_5
#define FLASH_MOSI_GPIO_Port GPIOB

#define PAIR_BT_PB1_Pin GPIO_PIN_1
#define PAIR_BT_PB1_GPIO_Port GPIOB
#define MODE_BT_PB10_Pin GPIO_PIN_10
#define MODE_BT_PB10_GPIO_Port GPIOB
#define RESET_BT_PB11_Pin GPIO_PIN_11
#define RESET_BT_PB11_GPIO_Port GPIOB

#define KEY_FUN_PG12_Pin GPIO_PIN_7         /*待机模式:蓝牙耳机切换*/
#define KEY_FUN_PG12_GPIO_Port GPIOD

#define KEY_ASK_PG10_Pin GPIO_PIN_10        /*播放模式:报告\待机模式:播放录音*/
#define KEY_ASK_PG10_GPIO_Port GPIOG

#define KEY_VOL_UP_PG9_Pin GPIO_PIN_9       /*播放模式:音量+ \待机模式:随机播放*/
#define KEY_VOL_UP_PG9_GPIO_Port GPIOG

#define KEY_VOL_DOWN_PG11_Pin GPIO_PIN_11   /*播放模式:音量- \待机模式:录音播放*/
#define KEY_VOL_DOWN_PG11_GPIO_Port GPIOG

#define KEY_WAKE_UP_PA0_Pin GPIO_PIN_0         /*蓝牙配对*/
#define KEY_WAKE_UP_PA0_GPIO_Port GPIOA

#define IIC_SCLK_PB8_Pin GPIO_PIN_8
#define IIC_SCLK_PB8_GPIO_Port GPIOB
#define IIC_SDA_PB9_Pin GPIO_PIN_9
#define IIC_SDA_PB9_GPIO_Port GPIOB

#define REF_EN_PC4_Pin GPIO_PIN_4
#define REF_EN_PC4_GPIO_Port GPIOC

#elif defined(F429_ZET6)

#define KEY_Pin GPIO_PIN_8
#define KEY_GPIO_Port GPIOI
#define FLASH_CS_Pin GPIO_PIN_3
#define FLASH_CS_GPIO_Port GPIOG
#define FLASH_SCK_Pin GPIO_PIN_3
#define FLASH_SCK_GPIO_Port GPIOB
#define FLASH_MISO_Pin GPIO_PIN_4
#define FLASH_MISO_GPIO_Port GPIOB
#define FLASH_MOSI_Pin GPIO_PIN_5
#define FLASH_MOSI_GPIO_Port GPIOB

#define PAIR_BT_PB1_Pin GPIO_PIN_1
#define PAIR_BT_PB1_GPIO_Port GPIOB
#define MODE_BT_PB10_Pin GPIO_PIN_10
#define MODE_BT_PB10_GPIO_Port GPIOB
#define RESET_BT_PB11_Pin GPIO_PIN_11
#define RESET_BT_PB11_GPIO_Port GPIOB

#define KEY_FUN_PG12_Pin GPIO_PIN_7         /*待机模式:蓝牙耳机切换*/
#define KEY_FUN_PG12_GPIO_Port GPIOD

#define KEY_ASK_PG10_Pin GPIO_PIN_10        /*播放模式:报告\待机模式:播放录音*/
#define KEY_ASK_PG10_GPIO_Port GPIOG

#define KEY_VOL_UP_PG9_Pin GPIO_PIN_9       /*播放模式:音量+ \待机模式:随机播放*/
#define KEY_VOL_UP_PG9_GPIO_Port GPIOG

#define KEY_VOL_DOWN_PG11_Pin GPIO_PIN_11   /*播放模式:音量- \待机模式:录音播放*/
#define KEY_VOL_DOWN_PG11_GPIO_Port GPIOG

#define KEY_WAKE_UP_PA0_Pin GPIO_PIN_0         /*蓝牙配对*/
#define KEY_WAKE_UP_PA0_GPIO_Port GPIOA

#define IIC_SCLK_PB8_Pin GPIO_PIN_8
#define IIC_SCLK_PB8_GPIO_Port GPIOB
#define IIC_SDA_PB9_Pin GPIO_PIN_9
#define IIC_SDA_PB9_GPIO_Port GPIOB

#define REF_EN_PC4_Pin GPIO_PIN_4
#define REF_EN_PC4_GPIO_Port GPIOC

#define LED_PD5_Pin GPIO_PIN_5
#define LED_PD5_GPIO_Port GPIOD

#endif
/* USER CODE BEGIN Private defines */
/* EVENTS GROUP */
#define EVENTS_VOL_UP_BIT                    (1UL <<  0UL)
#define EVENTS_VOL_DOWN_BIT                  (1UL <<  1UL)
#define EVENTS_ASK_BIT                       (1UL <<  2UL)
#define EVENTS_FUN_BLE_CHANGE_BIT            (1UL <<  3UL)
#define EVENTS_FUN_BLE_PAIR_BIT              (1UL <<  4UL)
#define EVENTS_FUN_STOP_BIT                  (1UL <<  5UL)
#define EVENTS_PLAY_BIT                      (1UL <<  6UL)
#define EVENTS_RECORD_BIT                    (1UL <<  7UL)
#define EVENTS_PLAY_AND_RECORD_BIT           (1UL <<  8UL)
#define EVENTS_PLAY_END_BIT                  (1UL <<  9UL)
#define EVENTS_RECORD_END_BIT                (1UL << 10UL)
#define EVENTS_PLAY_AND_RECORD_END_BIT       (1UL << 11UL)

/* QUEUE */
#define QUEUE_LOG_ITEM_SIZE                   70
#define QUEUE_LOG_ITEM_LENGTH                 10
/* file dir */

#define MUSIC_PATH              "0:/MUSIC/"
#define RECORD_PATH             "0:/RECORD/"
#define LOG_PATH                "0:/LOG/"

//判断按键工作状态
extern uint8_t key_work_status;

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);

/* USER CODE END Private defines */
/**@brief Macro for calling error handler function.
 *
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define APP_ERROR_HANDLER(ERR_CODE)                         \
    do                                                      \
    {                                                       \
        app_error_handler((ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
    } while (0)

/**@brief Macro for calling error handler function if supplied error code any other than NRF_SUCCESS.
 *
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define APP_ERROR_CHECK(ERR_CODE)                           \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != 0)                  \
        {                                                   \
            APP_ERROR_HANDLER(LOCAL_ERR_CODE);              \
        }                                                   \
    } while (0)

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#if defined (DEBUG_LOG)

#define app_trace_log(...)    printf(__VA_ARGS__)

#else

#define app_trace_log(...)

#endif
/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
