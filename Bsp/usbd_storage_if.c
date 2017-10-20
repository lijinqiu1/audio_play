/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @brief          : Memory management layer
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
#include "usbd_storage_if.h"
/* USER CODE BEGIN INCLUDE */
#include "bsp_driver_sd.h"
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @{
  */
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */
/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @{
  */
#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

/* USER CODE BEGIN PRIVATE_DEFINES */
__IO uint32_t writestatus, readstatus = 0;
extern SD_HandleTypeDef hsd;
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @{
  */
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_IF_Private_Variables
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA */
/* USB Mass storage Standard Inquiry Data */
const int8_t  STORAGE_Inquirydata[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
};
/* USER CODE END INQUIRY_DATA */
/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_IF_Exported_Variables
  * @{
  */
  extern USBD_HandleTypeDef HUSBDEVICE;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @{
  */

static int8_t STORAGE_Init (uint8_t lun);
static int8_t STORAGE_GetCapacity (uint8_t lun,
                           uint32_t *block_num,
                           uint16_t *block_size);
static int8_t  STORAGE_IsReady (uint8_t lun);
static int8_t  STORAGE_IsWriteProtected (uint8_t lun);
static int8_t STORAGE_Read (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len);
static int8_t STORAGE_Write (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len);
static int8_t STORAGE_GetMaxLun (void);
/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */


USBD_StorageTypeDef USBD_Storage_Interface_fops =
{
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  (int8_t *)STORAGE_Inquirydata,
};

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : STORAGE_Init
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Init (uint8_t lun)
{
  /* USER CODE BEGIN 9 */
  if(BSP_SD_Init() != 0)
  {
	  return USBD_FAIL;
  }
  return (USBD_OK);
  /* USER CODE END 9 */
}

/*******************************************************************************
* Function Name  : STORAGE_GetCapacity
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 10 */
  HAL_SD_CardInfoTypeDef info;
  int8_t ret = -1;

  if (BSP_SD_IsDetected() != SD_NOT_PRESENT)
  {
    BSP_SD_GetCardInfo(&info);

    *block_num = info.LogBlockNbr - 1;
    *block_size = info.LogBlockSize;
    ret = 0;
  }
  return ret;
  /* USER CODE END 10 */
}

/*******************************************************************************
* Function Name  : STORAGE_IsReady
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsReady (uint8_t lun)
{
  /* USER CODE BEGIN 11 */
	int8_t ret = -1;
	if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
	{
	  ret = 0;
	}
	return ret;
  /* USER CODE END 11 */
}

/*******************************************************************************
* Function Name  : STORAGE_IsWriteProtected
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsWriteProtected (uint8_t lun)
{
  /* USER CODE BEGIN 12 */
  return (USBD_OK);
  /* USER CODE END 12 */
}

/*******************************************************************************
* Function Name  : STORAGE_Read
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Read (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len)
{
  /* USER CODE BEGIN 13 */
  int8_t ret = -1;

  if (BSP_SD_IsDetected() != SD_NOT_PRESENT)
  {
#if defined (TEST)
//	if(BSP_SD_ReadBlocks((uint32_t *) buf, blk_addr, blk_len,HAL_MAX_DELAY)==0)
//		ret = 0;
    BSP_SD_ReadBlocks_DMA((uint32_t *) buf, blk_addr, blk_len);
    /* Wait for Rx Transfer completion */
    while (readstatus == 0)
    {
    }
    readstatus = 0;

    /* Wait until SD card is ready to use for new operation */
    while (BSP_SD_GetCardState() != SD_TRANSFER_OK)
    {
    }
    ret = 0;
#else
	if (BSP_SD_ReadBlocks_DMA((uint32_t *) buf, blk_addr, blk_len) == 0)
	{
		ret = 0;
	}
#endif
  }
  return ret;
  /* USER CODE END 13 */
}

/*******************************************************************************
* Function Name  : STORAGE_Write
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Write (uint8_t lun,
                         uint8_t *buf,
                         uint32_t blk_addr,
                         uint16_t blk_len)
{
  /* USER CODE BEGIN 14 */
  int8_t ret = -1;

  if (BSP_SD_IsDetected() != SD_NOT_PRESENT)
  {
#if defined (TEST)
//	if(BSP_SD_WriteBlocks((uint32_t *) buf, blk_addr, blk_len,HAL_MAX_DELAY)==0)
//		ret = 0;
    BSP_SD_WriteBlocks_DMA((uint32_t *) buf, blk_addr, blk_len);

    /* Wait for Tx Transfer completion */
    while (writestatus == 0)
    {
    }
    writestatus = 0;

    /* Wait until SD card is ready to use for new operation */
    while (BSP_SD_GetCardState() != SD_TRANSFER_OK)
    {
    }
#else
	if (BSP_SD_WriteBlocks_DMA((uint32_t *) buf, blk_addr, blk_len) == 0)
	{
		ret = 0;
	}
#endif
  }
  return ret;
  /* USER CODE END 14 */
}

/*******************************************************************************
* Function Name  : STORAGE_GetMaxLun
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetMaxLun (void)
{
  /* USER CODE BEGIN 15 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 15 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @brief BSP Tx Transfer completed callbacks
  * @param None
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void)
{
  writestatus = 1;
}

/**
  * @brief BSP Rx Transfer completed callbacks
  * @param None
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{
  readstatus = 1;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
