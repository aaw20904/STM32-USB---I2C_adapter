/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

extern   uint8_t i2cRxSlaveBuffer[260];
extern   uint8_t i2cTxSlaveBuffer[260];
extern volatile  uint16_t slaveI2cBufferLimit;

extern   uint8_t i2cTxMasterBuffer;
extern volatile wrp_i2c_master_header i2cMasterHeader;
extern volatile uint32_t adapterSemaphoreI2C;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN EV */
extern volatile wrp_i2c_slave_header i2cSlaveHeader;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB high priority or CAN TX interrupts.
  */
void USB_HP_CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 0 */

  /* USER CODE END USB_HP_CAN1_TX_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 1 */

  /* USER CODE END USB_HP_CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

	uint16_t tempVar;

			if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_ADDR) {
		      //Address sent (master mode)/matched (slave mode)
		       //clear flag
				tempVar = I2C1->SR2;

				if (i2cSlaveHeader.hDevice->SR2 & I2C_SR2_TRA) {
					///slave transmitter mode
					GPIOC->BSRR = GPIO_BSRR_BR13; //turn led on Tx (optional)
					i2cSlaveHeader.i2cTxSlaveIndex = 0;
				} else {
					//slave receiver mode
					GPIOC->BSRR = GPIO_BSRR_BR13; //turn on led Rx (optional)
					i2cSlaveHeader.i2cRxSlaveIndex = 0;
				}



			}
			if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_RXNE) {
		     //Data register not empty (receivers)
				 //NOTE:clear flag when a user reads data
				//read data
				i2cSlaveHeader.i2cRxSlaveBuffer[i2cSlaveHeader.i2cRxSlaveIndex] = I2C1->DR;
				i2cSlaveHeader.i2cRxSlaveIndex++;
		       ///for safety
				if (i2cSlaveHeader.i2cRxSlaveIndex >  i2cSlaveHeader.slaveI2cBufferLimit){
					i2cSlaveHeader.i2cRxSlaveIndex=0;
				}
			}

			if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_TXE) {
		     ///Data register empty (transmitters)
				//interrupt fires firstly when the address has been sent
				//and affer each byte
				i2cSlaveHeader.hDevice->DR =i2cSlaveHeader.i2cTxSlaveBuffer[i2cSlaveHeader.i2cTxSlaveIndex];
				i2cSlaveHeader.i2cTxSlaveIndex++;
				//for safety
				if (i2cSlaveHeader.i2cTxSlaveIndex >  i2cSlaveHeader.slaveI2cBufferLimit){
					i2cSlaveHeader.i2cTxSlaveIndex=0;
				}
			}

			if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_STOPF) {
		     //Stop detection receiver (slave mode)
				tempVar = I2C1->SR2; //read SR1 and SR2 (as in ref manual)
				i2cSlaveHeader.hDevice->CR1 |= I2C_CR1_PE; //write ANY bit inside CR! (ass in ref manual)
				GPIOC->BSRR = GPIO_BSRR_BS13; //turn LED off
			}

			(void)tempVar;

  /* USER CODE END I2C1_EV_IRQn 0 */

  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */
	 if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_ADDR) {
				//turn the led ON (optional)
		       GPIOC->BSRR = GPIO_BSRR_BR13;
		    }

		    //save error states inside a global state variable
		 i2cSlaveHeader.i2cSlaveErrorFlags = i2cSlaveHeader.hDevice->SR2;
			//clerar error flags
		 i2cSlaveHeader.i2cSlaveErrorFlags &= (I2C_SR1_BERR|I2C_SR1_ARLO|I2C_SR1_OVR);
		    	if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_AF) {
		    			//end byte  of the slave transmitter (the last byte NACKed)
		    			//clear flag
		    		i2cSlaveHeader.hDevice->SR1 &= ~I2C_SR1_AF;
					   //turn the LED off (optional)
		    			GPIOC->BSRR = GPIO_BSRR_BS13;
		    	}

		    	if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_ARLO) {
		    		//clear flag arbitration lost
		    		i2cSlaveHeader.hDevice->SR1 &= ~I2C_SR1_ARLO;
		    	}

		    	if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_BERR) {
		    		//clear flag bus error
		    		i2cSlaveHeader.hDevice->SR1 &= ~I2C_SR1_BERR;
		    	}

		    	if (i2cSlaveHeader.hDevice->SR1 & I2C_SR1_OVR) {
		    		//clear flag overrun/underrun
		    		i2cSlaveHeader.hDevice->SR1 &= ~I2C_SR1_OVR;
		    	}
  /* USER CODE END I2C1_ER_IRQn 0 */

  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */
			uint16_t tempVar, remainder;
		  /* USER CODE BEGIN I2C2_EV_IRQn 0 */
			if (i2cMasterHeader.hDevice->SR1 & I2C_SR1_SB) {
				//START event has been ended

				//1)Write slave address

				 i2cMasterHeader.i2cRxMasterIndex = 0;
				 i2cMasterHeader.i2cTxMasterIndex = 0;

				 //2)Enable acknowledge
				 i2cMasterHeader.hDevice->CR1 |= I2C_CR1_ACK;
				 //3)write address - (b1-b7 address, b0 R/W)
				tempVar = (i2cMasterHeader.i2cMasterAddress << 1);

				 if (i2cMasterHeader.WriteFlag == 0) {
				  //4)master receiver mode - when WriteFlag = 0
					  tempVar |= 1;
				 }
				 //5)write temporary variable to dataRegister and start address transmission
	                 i2cMasterHeader.hDevice->DR = tempVar;
			}

			if (i2cMasterHeader.hDevice->SR1 & I2C_SR1_ADDR) {
			      //Address has been  sent to the slave device

		            //init pointers-indexes
					i2cMasterHeader.i2cRxMasterIndex = 0;
					i2cMasterHeader.i2cTxMasterIndex = 0;
	                  //what is the I2C action type? (R/W)
					if (i2cMasterHeader.hDevice->SR2 & I2C_SR2_TRA) {
						/// ***master T R A N S M I T T E R mode***
						 //2)clear ADDR flag by reading SR2 reg
							tempVar = I2C2->SR2;
						//turn led on Tx (optional)
						GPIOC->BSRR = GPIO_BSRR_BR13;
					} else {
						//***master R E C E I V E R mode***
						//turn on led Rx (optional)
						 GPIOC->BSRR = GPIO_BSRR_BR13;
						if (i2cMasterHeader.i2cMasterRxPacketLength == 1) {
							  ///-----PACKET LENGTH = 1--------
							  //when one byte reception - clear ACK bit
							i2cMasterHeader.hDevice->CR1 &= ~I2C_CR1_ACK;
							    //clear address flag by reading SR1 and SR2:
							  tempVar = I2C2->SR2;
							    //and prepare stop
						    i2cMasterHeader.hDevice->CR1 |= I2C_CR1_STOP;
						} else if (i2cMasterHeader.i2cMasterRxPacketLength == 2){
							  ///------PACKET LENGTH = 2--------
							  //clear address flag by reading SR1 and SR2:
								tempVar = I2C2->SR2;
							//disable ACK
							 i2cMasterHeader.hDevice->CR1 &= ~I2C_CR1_ACK;
							 //enable POS
							 i2cMasterHeader.hDevice->CR1 |= I2C_CR1_POS;
							 //2)clear ADDR flag by reading SR2 reg

						} else if (i2cMasterHeader.i2cMasterRxPacketLength >= 3){
							  ///------PACKET LENGTH >= 3--------
							//2)clear ADDR flag by reading SR2 reg
							tempVar = I2C2->SR2;
						}

					}

			}

			if (i2cMasterHeader.hDevice->SR1 & I2C_SR1_TXE) {

				tempVar = I2C2->SR2;
			     ///Data register empty (transmitters)
					//interrupt also fires firstly when the address has been sent
					//and affer each byte
				if (i2cMasterHeader.i2cTxMasterIndex >= i2cMasterHeader.i2cMasterTxPacketLength) {
					//when the last byte of a packet
					//generate STOP condition
					i2cMasterHeader.hDevice->CR1 |= I2C_CR1_STOP;
					i2cMasterHeader.hDevice->DR = 0xff; //clear flag
					//OPTIONAL: clear semaphore:
					adapterSemaphoreI2C &= ~semaphore_await_tx_i2c;
				} else {
		           //write new data into data register
					i2cMasterHeader.hDevice->DR =i2cMasterHeader.i2cTxMasterBuffer[i2cMasterHeader.i2cTxMasterIndex];
					i2cMasterHeader.i2cTxMasterIndex++;
				}

			}

			if (i2cMasterHeader.hDevice->SR1 & I2C_SR1_RXNE) {
				//calculate a remainder
				remainder = i2cMasterHeader.i2cMasterRxPacketLength - i2cMasterHeader.i2cRxMasterIndex;

				if (i2cMasterHeader.i2cMasterRxPacketLength == 1) {
					///-----PACKET LENGTH = 1--------
					//just read data, STOP had been set in ADDR event handler
					 i2cMasterHeader.i2cRxMasterBuffer[i2cMasterHeader.i2cRxMasterIndex] = i2cMasterHeader.hDevice->DR;
						//OPTIONAL: clear semaphore:
						adapterSemaphoreI2C &= ~semaphore_await_rx_i2c;

				} else if ((i2cMasterHeader.i2cMasterRxPacketLength == 2) && (i2cMasterHeader.hDevice->SR1 & I2C_SR1_BTF)) {
					///-----PACKET LENGTH = 2--------
					//1)programming the STOP
					  i2cMasterHeader.hDevice->CR1 |= I2C_CR1_STOP;
					//2)read DR twice in sequence:
					 i2cMasterHeader.i2cRxMasterBuffer[i2cMasterHeader.i2cRxMasterIndex] = i2cMasterHeader.hDevice->DR;
					 i2cMasterHeader.i2cRxMasterIndex++;
					 i2cMasterHeader.i2cRxMasterBuffer[i2cMasterHeader.i2cRxMasterIndex] = i2cMasterHeader.hDevice->DR;
					//3)disable POS
					 i2cMasterHeader.hDevice->CR1 &= ~I2C_CR1_POS;
						//OPTIONAL: clear semaphore:
						adapterSemaphoreI2C &= ~semaphore_await_rx_i2c;
				} else if ((remainder > 3) && (i2cMasterHeader.i2cMasterRxPacketLength >=3 )) {
					///-----PACKET LENGTH >= 3
					///1) Read data from 0 t0 PacketLength-3
					  i2cMasterHeader.i2cRxMasterBuffer[i2cMasterHeader.i2cRxMasterIndex] = i2cMasterHeader.hDevice->DR;
					  i2cMasterHeader.i2cRxMasterIndex++;
				} else if ( (i2cMasterHeader.hDevice->SR1 & I2C_SR1_BTF) && (i2cMasterHeader.i2cMasterRxPacketLength >=3) ) {
					//------PACKET LENGTH >= 3-----read data PacketLen-2,PacketLen-1-----
                     //1) NACK
					   i2cMasterHeader.hDevice->CR1 &= ~I2C_CR1_ACK;
					 //2)Read PacketLen-2 data
						 i2cMasterHeader.i2cRxMasterBuffer[i2cMasterHeader.i2cRxMasterIndex] = i2cMasterHeader.hDevice->DR;
						 i2cMasterHeader.i2cRxMasterIndex++;
					//3)Send STOP
						 i2cMasterHeader.hDevice->CR1 |= I2C_CR1_STOP;
					 //2)Read PacketLen-1 data
						 i2cMasterHeader.i2cRxMasterBuffer[i2cMasterHeader.i2cRxMasterIndex] = i2cMasterHeader.hDevice->DR;
						 i2cMasterHeader.i2cRxMasterIndex++;
				} else if ((remainder == 1) && (i2cMasterHeader.i2cMasterRxPacketLength >=3 )) {
					 ////------PACKET LENGTH >= 3-----read the last byte PacketLegth-0
						i2cMasterHeader.i2cRxMasterBuffer[i2cMasterHeader.i2cRxMasterIndex] = i2cMasterHeader.hDevice->DR;
						i2cMasterHeader.i2cRxMasterIndex++;
						//OPTIONAL: clear semaphore:
						adapterSemaphoreI2C &= ~semaphore_await_rx_i2c;

				}



			}
			(void)tempVar;

  /* USER CODE END I2C2_EV_IRQn 0 */

  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */
	//save flags
	i2cMasterHeader.i2cMasterErrorFlags = i2cMasterHeader.hDevice->SR1;
		i2cMasterHeader.i2cMasterErrorFlags &= (I2C_SR1_BERR|I2C_SR1_ARLO|I2C_SR1_OVR|I2C_SR1_AF);
			if (i2cMasterHeader.hDevice->SR1 & I2C_SR1_AF) {
				//end byte  of the slave transmitter (the last byte NACKed)

				//clear flag
				i2cMasterHeader.hDevice->SR1 &= ~I2C_SR1_AF;
				//3)Send STOP
					i2cMasterHeader.hDevice->CR1 |= I2C_CR1_STOP;
					//OPTIONAL: clear semaphore:
					adapterSemaphoreI2C &= ~semaphore_await_tx_i2c;
					adapterSemaphoreI2C &= ~semaphore_await_rx_i2c;
				//GPIOB->BSRR = GPIO_BSRR_BR14;
			}

			if (i2cMasterHeader.hDevice->SR1 & I2C_SR1_ARLO){
				//clear flag arbitration lost
				i2cMasterHeader.hDevice->SR1 &= ~I2C_SR1_ARLO;
				//3)Send STOP
				i2cMasterHeader.hDevice->CR1 |= I2C_CR1_STOP;
				//OPTIONAL: clear semaphore:
				adapterSemaphoreI2C &= ~semaphore_await_tx_i2c;
				adapterSemaphoreI2C &= ~semaphore_await_rx_i2c;
			}

			if (i2cMasterHeader.hDevice->SR1 & I2C_SR1_BERR){
				//clear flag bus error
				I2C2->SR1 &= ~I2C_SR1_BERR;
				//3)Send STOP
				i2cMasterHeader.hDevice->CR1 |= I2C_CR1_STOP;
				//OPTIONAL: clear semaphore:
				adapterSemaphoreI2C &= ~semaphore_await_tx_i2c;
				adapterSemaphoreI2C &= ~semaphore_await_rx_i2c;
			}

			if (i2cMasterHeader.hDevice->SR1 & I2C_SR1_OVR){
				//clear flag overrun/underrun
				I2C2->SR1 &= ~I2C_SR1_OVR;
				//3)Send STOP
				i2cMasterHeader.hDevice->CR1 |= I2C_CR1_STOP;
				//OPTIONAL: clear semaphore:
				adapterSemaphoreI2C &= ~semaphore_await_tx_i2c;
				adapterSemaphoreI2C &= ~semaphore_await_rx_i2c;
			}
			//Clear all the flags:

  /* USER CODE END I2C2_ER_IRQn 0 */

  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
