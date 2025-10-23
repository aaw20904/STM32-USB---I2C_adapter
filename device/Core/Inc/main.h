/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef   struct {
	 I2C_TypeDef * hDevice;
	  uint16_t  i2cSlaveRxPacketLength;
	  uint16_t  i2cSlaveTxPacketLength;
	  uint16_t  slaveI2cBufferLimit;
	  uint16_t  i2cRxSlaveIndex;
	  uint16_t  i2cTxSlaveIndex;
	  uint32_t  i2cSlaveErrorFlags;
	  uint8_t * i2cRxSlaveBuffer;
	  uint8_t * i2cTxSlaveBuffer;
	  uint8_t   i2cSlaveAddress;
	  DMA_Channel_TypeDef * dmaRxChanel;
	  DMA_Channel_TypeDef * dmaTxChanel;

} wrp_i2c_slave_header;

typedef   struct {
	 I2C_TypeDef * hDevice;
	  uint16_t i2cMasterRxPacketLength;
	  uint16_t i2cMasterTxPacketLength;
	  uint16_t  masterI2cBufferLimit;
	  uint16_t i2cRxMasterIndex;
	  uint16_t i2cTxMasterIndex;
	  uint32_t i2cMasterErrorFlags;
	  uint8_t * i2cRxMasterBuffer;
	  uint8_t * i2cTxMasterBuffer;
	  uint8_t i2cMasterAddress;
	  DMA_Channel_TypeDef * dmaRxChanel;
	  DMA_Channel_TypeDef * dmaTxChanel;
	  uint8_t WriteFlag;

} wrp_i2c_master_header;
///defines for state mashine

typedef struct {
		uint8_t typeOfAction;
		uint8_t* buffPtr; //changes during exec.
		uint16_t numUsbTransations;
		uint16_t numBytesToTransaction;
		uint8_t*  reassembledDataArray; //not changed
		uint8_t slaveAddress;
} statesHandle;




#define semaphore_await_rx_usb  1
#define semaphore_await_tx_usb  2
//----------------------------------
#define semaphore_await_rx_i2c  1
#define semaphore_await_tx_i2c  2
///---------data markers-------------
#define  data_from_host   18
#define data_to_host  19
//-----------commands----------------
#define write_to_i2c_dev  24
#define read_from_i2c_dev 25
#define reset_interface 26
#define setup_interface_i2c 27
#define read_last_stub_rx_i2c 28
#define write_tx_stub_buffer 29
//---error codes-------------------
#define adapter_success 0
#define adapter_AF 1
#define adapter_BERR 2
#define adapter_ARLO 3
#define adapter_OVR 4
#define adapter_timeout 5
#define adapter_other_error 6
#define adapter_busy 7

#define REPORT_ID 1

#define PAYLOAD_OFFS 4



int32_t readI2cToUsb (statesHandle* pToStates);
int32_t adapterClear (statesHandle* pToStates);



void USB_CDC_RxHandler(uint8_t*, uint32_t);

void i2c_usr_master_init(volatile wrp_i2c_master_header* header);


void i2c_usr_slave_init (volatile wrp_i2c_slave_header*   header);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
