/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**

█░░ █ █▀▄▀█ █ ▀█▀ ▄▀█ ▀█▀ █ █▀█ █▄░█ █▀ ░   █▀▀ █▀█ █▄░█ █▀ ▀█▀ █▀█ ▄▀█ █ █▄░█ ▀█▀ █▀
█▄▄ █ █░▀░█ █ ░█░ █▀█ ░█░ █ █▄█ █░▀█ ▄█ █   █▄▄ █▄█ █░▀█ ▄█ ░█░ █▀▄ █▀█ █ █░▀█ ░█░ ▄█
maximum data length of the Master (I2C2) = 1024bytes
maximum data Length of the Slave = 256bytes (when more - wrap around)
*/
/*I M P O R T A N T : Set the
  #define USBD_CUSTOM_HID_REPORT_DESC_SIZE     38
  in the usbd_conf.h file if a PC not see your device
  */
// USB externs
extern USBD_HandleTypeDef hUsbDeviceFS;
//I2C haeders
volatile wrp_i2c_slave_header i2cSlaveHeader;
volatile wrp_i2c_master_header i2cMasterHeader;

  unsigned char usb_buffer_for_report_dev[68];
  unsigned char usb_buffer_for_report_host[68];
//volatile unsigned char usb_buffer_for_report[68];

  uint8_t i2cRxSlaveBuffer[256];
  uint8_t i2cTxSlaveBuffer[256]= { "Alice was beginning to get very tired of sitting by her sister on the bank, and of having nothing to do: once or twice she had peeped into the book her sister was reading, but it had no pictures or conversations in it, 'and what is the use of a book,'.."};

  uint8_t i2cRxMasterBuffer[1024]={0};
  uint8_t i2cTxMasterBuffer[1204]={0};


LL_I2C_InitTypeDef I2C_InitStructSlave = {0};

LL_I2C_InitTypeDef I2C_InitStructMaster = {0};


//USB semaphore
volatile uint32_t adapterSemaphoreUSB;
//I2C semaphore
volatile uint32_t adapterSemaphoreI2C;

 statesHandle i2cAdapterStates;

/* USER CODE END PTD */

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
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */


//a****callback***for*****USB**********CDC***
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len) {
	// Copy data from USB internal buffer
	memcpy (usb_buffer_for_report_dev, Buf, Len);
	 //clear a semaphore
	adapterSemaphoreUSB &= ~semaphore_await_rx_usb;

}

/* H E L P:
 (1) When a Compiler doesn`t see ""usbd_customhid.h" file, right click on
 Project Explorer->Properties->MCU GCC Compiler->Include paths
 and add "../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc"
 (2) To add USB write complete callback,
   (A)open the "usbd_cdc_if.c" file
   and insert declaration:
   	   	   __weak void CDC_TxCpltCallback(void) {

   	   	   }
   (B)Call this function in ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c file
   in function: static uint8_t  USBD_CDC_DataIn()
  .....
  else
    {
      hcdc->TxState = 0U;
     CDC_TxCpltCallback();  //call here!

    }
    (C) Re-define this function in main.c:
    void CDC_TxCpltCallback(void) {
    adapterSemaphoreUSB &= ~semaphore_await_tx_usb;
}

 * */
void CDC_TxCpltCallback (void) {
    adapterSemaphoreUSB &= ~semaphore_await_tx_usb;
}

void i2c_usr_slave_init (volatile wrp_i2c_slave_header*   header) {
	 //disable I2C
	 header->hDevice->CR1 &= ~I2C_CR1_PE;
  //write own address
	 header->hDevice->OAR1 = header->i2cSlaveAddress;
  //enable acknowledgements
	 header->hDevice->CR1 |=  I2C_CR1_ACK;
	 //enable event interrupts and bufer interrupts
	 header->hDevice->CR2 |= I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN|I2C_CR2_ITERREN;
	  //turn I2C on, enable acknowledge
	 header->hDevice->CR1 |= I2C_CR1_PE|I2C_CR1_ACK;
}

void i2c_usr_master_init (volatile wrp_i2c_master_header* header) {
	 //enable own address
	 header->hDevice->CR1 &= ~I2C_CR1_PE;
	  //own address equals zero - master
	 header->hDevice->CR1 |=  I2C_CR1_ACK;
	 //enable event interrupts and bufer interrupts
	 header->hDevice->CR2 |= I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN|I2C_CR2_ITERREN;
	  //turn I2C on, enable acknowledge
	 header->hDevice->CR1 |= I2C_CR1_PE|I2C_CR1_ACK;

}


void resetI2cSetSpeedMaster (volatile wrp_i2c_master_header* mHeader, LL_I2C_InitTypeDef *mInitStruct, uint32_t speed) {

      //reset master
         I2C2->CR1 |= I2C_CR1_SWRST;
		   HAL_Delay(2);
		   I2C2->CR1  = 0;
			///set speed
		   mInitStruct->ClockSpeed = speed;
		 LL_I2C_Init(I2C2, mInitStruct);
		 i2c_usr_master_init(mHeader);

}

void resetI2cSetSpeedSlave (volatile wrp_i2c_slave_header* sHeader, LL_I2C_InitTypeDef *sInitStruct, uint32_t speed, uint8_t addr ) {
	//reset slave
	        I2C1->CR1 |= I2C_CR1_SWRST;
	      	   HAL_Delay(2);
	      	   I2C1->CR1  = 0;
	      	   	///set speed
	      	   sInitStruct->ClockSpeed = speed;
	      	   sInitStruct->OwnAddress1 = addr;
	      	   sHeader->i2cSlaveAddress = addr;
	      	 LL_I2C_Init(I2C1, sInitStruct);
	      	 i2c_usr_slave_init(sHeader);
}

void greenLedOn (void) {
	GPIOC->BSRR = GPIO_BSRR_BR13;
}

void greenLedOff (void) {
	GPIOC->BSRR = GPIO_BSRR_BS13;
}


uint8_t writeI2cAndWaitEnd (volatile wrp_i2c_master_header* i2cHeader, unsigned char address,   unsigned char* bufferPtr, unsigned short amountOfData) {
	//for safety:
				if (amountOfData >= i2cHeader->masterI2cBufferLimit){
					amountOfData = i2cHeader->masterI2cBufferLimit;
				}
   //clean old saved flags:
	i2cHeader->i2cMasterErrorFlags = 0;
 //set memory address
	i2cHeader->i2cTxMasterBuffer = bufferPtr;
  //set slave address-:
	i2cHeader->i2cMasterAddress = address;
  //set write mode:
	i2cHeader->WriteFlag = 1;
  //set amount of data:
	i2cHeader->i2cMasterTxPacketLength = amountOfData;
  //set the semaphore
	adapterSemaphoreI2C |= semaphore_await_tx_i2c;
	//start transaction:
	i2cHeader->hDevice->CR1 |= I2C_CR1_START;
	//wait until all the data will be sent. Now is time to send STOP command:
	while (adapterSemaphoreI2C & semaphore_await_tx_i2c){}
	//await until busy to stop sequence complete:
	while (i2cHeader->hDevice->SR2 & I2C_SR2_BUSY){}
	//checking for errors:
	switch (i2cHeader->i2cMasterErrorFlags) {
	case I2C_SR1_AF:
		return adapter_AF;
		break;
	case I2C_SR1_BERR:
		return adapter_BERR;
	     break;
	case I2C_SR1_ARLO:
		return adapter_ARLO;
		break;
	case I2C_SR1_OVR:
		return adapter_OVR;
		 break;
	default:
		return 0;
	}


}

int readI2cAndWaitEnd (volatile wrp_i2c_master_header* i2cHeader, unsigned char address,   unsigned char* bufferPtr, unsigned short amountOfData) {
	//for safety
	if (amountOfData >= i2cHeader->masterI2cBufferLimit){
						amountOfData = i2cHeader->masterI2cBufferLimit;
	}
	//clean old saved flags:
		i2cHeader->i2cMasterErrorFlags = 0;
	 //set memory address
		i2cHeader->i2cRxMasterBuffer = bufferPtr;
	  //set slave address-:
		i2cHeader->i2cMasterAddress = address;
	  //set receiver mode
		i2cHeader->WriteFlag = 0;
	  //set amount of data:
		i2cHeader->i2cMasterRxPacketLength = amountOfData;
	  //set the semaphore
		adapterSemaphoreI2C |= semaphore_await_rx_i2c;
		//start transaction:
		i2cHeader->hDevice->CR1 |= I2C_CR1_START;
		//wait until all the data will be sent. Now is time to send STOP command:
		while (adapterSemaphoreI2C & semaphore_await_rx_i2c){}
		//await until busy to stop sequence complete:
		while (i2cHeader->hDevice->SR2 & I2C_SR2_BUSY){}
		//checking for errors:
		switch (i2cHeader->i2cMasterErrorFlags) {
		case I2C_SR1_AF:
			return adapter_AF;
			break;
		case I2C_SR1_BERR:
			return adapter_BERR;
		     break;
		case I2C_SR1_ARLO:
			return adapter_ARLO;
			break;
		case I2C_SR1_OVR:
			return adapter_OVR;
			 break;
		default:
			return 0;
		}
  return 0;
}





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* NOTE: number of data to be sent should be written in pToStates->numBytesToTransaction
*/

void sendReportToHost(    uint8_t* oneFrameBuffer){
	      //1)set the semaphore - await end of transmission
		adapterSemaphoreUSB |= semaphore_await_tx_usb;
		  //2)send data
		CDC_Transmit_FS(oneFrameBuffer, 64);

		  //4)await until transaction was finished
		while (adapterSemaphoreUSB & semaphore_await_tx_usb){/*do nothing*/}
}

void awaitReportFromHost (void) {
	 //1)set the semaphore - await end of transmission
			adapterSemaphoreUSB |= semaphore_await_rx_usb;
			  //4)await until transaction was finished
		while (adapterSemaphoreUSB & semaphore_await_rx_usb){/*do nothing*/}
}

void setControlForNewHostData (void) {
	//1)set the semaphore - await end of transmission
		adapterSemaphoreUSB |= semaphore_await_rx_usb;
}

void checkForNewHostDataOrAwait (void) {
	  //4)await until transaction was finished
		while (adapterSemaphoreUSB & semaphore_await_rx_usb){/*do nothing*/}
}

int32_t readUsbFromI2c (statesHandle* pToStates,
						  uint8_t* oneFrameBufferDev,
						  uint8_t* oneFrameBufferHost) {

	unsigned char temp=0;
	unsigned char* tempCharPtr =0;
	pToStates->typeOfAction = data_to_host;
	//1)Read data from I2C slave:
	temp = readI2cAndWaitEnd(&i2cMasterHeader, pToStates->slaveAddress, pToStates->reassembledDataArray, pToStates->numBytesToTransaction);
	//2)Send returned status  to a Host:
	oneFrameBufferHost[0]  = temp;
	sendReportToHost(oneFrameBufferHost);

	//3)When the transaction fail - return
	if (temp != 0) {
		return -1;
	}
	//4)When a transaction successfull - wait a request 'data_to_host' from a host

	/*3)Initialize the pointer (it was changed in previous transaction  */
	tempCharPtr = pToStates->reassembledDataArray;
	pToStates->buffPtr = tempCharPtr;
	//5)Send chunks to a host consequently:
	/*Iterate, until str.numUsbTransations == 1 */
		while (pToStates->numUsbTransations > 1) {
			awaitReportFromHost();
			if (oneFrameBufferDev[0] != data_to_host)
			   {
					//when incorrect request from a host - return error
					oneFrameBufferHost[0] = adapter_other_error;
					return -1;
			   }
			  //4.1)copy 64 bytes from I2C buffer
			memcpy(oneFrameBufferHost, pToStates->buffPtr, 64);
			pToStates->buffPtr += 64;
			  //4.3)decrease number of transactions:
			pToStates->numUsbTransations--;
			  //4.4)decrease amount of bytes:
			pToStates->numBytesToTransaction -= 64;
			  //4.5)Response to the host with a data packet
			sendReportToHost( oneFrameBufferHost);


		}
		//there is only one piece of data
		 //5) Waiting new data from a host
		awaitReportFromHost();
				if (oneFrameBufferDev[0] != data_to_host) {
						//when incorrect request from a host - return error
						oneFrameBufferHost[0] = adapter_other_error;
						return -1;
				}
		  /*6) copy last data piece into the buffer */
			memcpy(oneFrameBufferHost ,pToStates->buffPtr,  pToStates->numBytesToTransaction);
			 //9)Response to the host with the last  data packet:
			sendReportToHost( oneFrameBufferHost);
           return 0;

}

int32_t writeUsbSlaveTxBuffer(  statesHandle* pToStates,
								uint8_t* oneFrameBufferDev,
								 uint8_t* oneFrameBufferHost
								 ) {

		unsigned char* tempCharPtr =0;
		pToStates->typeOfAction = data_from_host;

		/*3)Initialize the pointer (it was changed in previous transaction  */
		tempCharPtr = pToStates->reassembledDataArray;
		pToStates->buffPtr = tempCharPtr;
		/*4)Iterate, until str.numUsbTransations == 1 */
		while (pToStates->numUsbTransations > 1) {
			/*4.1)Send to a host request 'data_from_host':*/
			oneFrameBufferHost[0] = data_from_host;
			sendReportToHost(oneFrameBufferHost);

			/*4.2)await new message from the host (data will be stored in the 'oneFrameBuffer'):*/
			awaitReportFromHost();
			  //4.3)copy 64 bytes into I2C buffer
			memcpy(pToStates->buffPtr, oneFrameBufferDev, 64);
			pToStates->buffPtr += 64;
			  //4.4)decrease number of transactions:
			pToStates->numUsbTransations--;
			  //4.5)decrease amount of bytes:
			pToStates->numBytesToTransaction -= 64;


		}
		/*5)When there is only one transaction, request the last data from a host:
		 */
		oneFrameBufferHost[0] = data_from_host;
		sendReportToHost(oneFrameBufferHost);
		awaitReportFromHost();
		memcpy(pToStates->buffPtr, oneFrameBufferDev, pToStates->numBytesToTransaction);
       return 0;
}


int32_t readUsbLastReadDataOfSlave(statesHandle* pToStates,
								  uint8_t* oneFrameBufferDev,
								  uint8_t* oneFrameBufferHost){


	pToStates->typeOfAction = data_to_host;
	pToStates->buffPtr = pToStates->reassembledDataArray;
	//calculate amount of transactions

	//send success and data size:
	oneFrameBufferHost[0] = 0;
	oneFrameBufferHost[1] = i2cSlaveHeader.i2cRxSlaveIndex & 0x00ff;
	oneFrameBufferHost[2] = (i2cSlaveHeader.i2cRxSlaveIndex & 0xff00) >> 8;
	//2)Send OK (byte0) and amount of data (byte1, 2)
	sendReportToHost(oneFrameBufferHost);

	//3)write data to host in sequence
	/*Iterate, until str.numUsbTransations == 1 */
		while (pToStates->numUsbTransations > 1) {
			awaitReportFromHost();
			if (oneFrameBufferDev[0] != data_to_host)
			   {
					//when incorrect request from a host - return error
					oneFrameBufferHost[0] = adapter_other_error;
					return -1;
			   }
			  //4.1)copy 64 bytes from I2C buffer
			memcpy(oneFrameBufferHost, pToStates->buffPtr, 64);
			pToStates->buffPtr += 64;
			  //4.3)decrease number of transactions:
			pToStates->numUsbTransations--;
			  //4.4)decrease amount of bytes:
			pToStates->numBytesToTransaction -= 64;
			  //4.5)Response to the host with a data packet
			sendReportToHost( oneFrameBufferHost);


		}
		//there is only one piece of data
		 //5) Waiting new data from a host
		awaitReportFromHost();
				if (oneFrameBufferDev[0] != data_to_host) {
						//when incorrect request from a host - return error
						oneFrameBufferHost[0] = adapter_other_error;
						return -1;
				}
		  /*6) copy last data piece into the buffer */
			memcpy(oneFrameBufferHost ,pToStates->buffPtr,  pToStates->numBytesToTransaction);
			 //9)Response to the host with the last  data packet:
			sendReportToHost( oneFrameBufferHost);
           return 0;

}



int32_t writeUsbToI2c (statesHandle* pToStates,
		  uint8_t* oneFrameBufferDev,
		  uint8_t* oneFrameBufferHost) {
		  uint16_t amountOfDataToProcessing =  pToStates->numBytesToTransaction;
		  unsigned char temp=0;
		  unsigned char* tempCharPtr =0;
	pToStates->typeOfAction = data_from_host;

	/*3)Initialize the pointer (it was changed in previous transaction  */
	tempCharPtr = pToStates->reassembledDataArray;
	pToStates->buffPtr = tempCharPtr;
	/*4)Iterate, until str.numUsbTransations == 1 */
	while (pToStates->numUsbTransations > 1) {
		/*4.1)Send to a host request 'data_from_host':*/
		oneFrameBufferHost[0] = data_from_host;
		sendReportToHost(oneFrameBufferHost);

		/*4.2)await new message from the host (data will be stored in the 'oneFrameBuffer'):*/
		awaitReportFromHost();
		  //4.3)copy 64 bytes into I2C buffer
		memcpy(pToStates->buffPtr, oneFrameBufferDev, 64);
		pToStates->buffPtr += 64;
		  //4.4)decrease number of transactions:
		pToStates->numUsbTransations--;
		  //4.5)decrease amount of bytes:
		pToStates->numBytesToTransaction -= 64;


	}
	/*5)When there is only one transaction, request the last data from a host:
	 */
	oneFrameBufferHost[0] = data_from_host;
	sendReportToHost(oneFrameBufferHost);

	/*6)await new message from the host (data will be stored in the 'oneFrameBuffer'):*/
	awaitReportFromHost();
	/*7) copy remainder into the buffer */
	memcpy(pToStates->buffPtr, oneFrameBufferDev , pToStates->numBytesToTransaction);
    //9)Restore value amount of bytes  for a transaction:
	pToStates->numBytesToTransaction = pToStates->buffPtr - pToStates->reassembledDataArray;
	//10)Start I2C transaction:
	temp = writeI2cAndWaitEnd( &i2cMasterHeader, pToStates->slaveAddress, pToStates->reassembledDataArray, amountOfDataToProcessing );
	//11)Send result of operation to the host:
	oneFrameBufferHost[0]= temp;
	sendReportToHost( oneFrameBufferHost);
    pToStates->typeOfAction = 0;
    return 0;
}


//----------------------------------------------------------------------------
int32_t commandDispatcher (statesHandle* pToStates,
							  uint8_t* oneFrameBufferHost, //data buffer for the Host (outgoing)
							  uint8_t* oneFrameBufferDev   //data buffer for device (incoming)
							) {
	uint32_t temp32_1;
	uint8_t temp8_1;
  //is the state mashine busy?
	if (pToStates->typeOfAction != 0) {
      //write error code in report
		oneFrameBufferHost[0] = adapter_busy;
		sendReportToHost(oneFrameBufferHost);
		 greenLedOff();
		return -2;
	}
	greenLedOn();
	//2)what is the command?
	switch (oneFrameBufferDev[0]) {
	case write_to_i2c_dev:
		//write operation
		//1)Write data from one-frame buffer into a structure:

		  pToStates->typeOfAction = oneFrameBufferDev[0];
		  pToStates->numBytesToTransaction = (oneFrameBufferDev[2] << 8) | oneFrameBufferDev[1];
		  pToStates->slaveAddress = oneFrameBufferDev[3];
		//2)amount of transactions (roughly):
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction / 64;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction % 64) > 0) {
			pToStates->numUsbTransations++;
		}
		//4)Initialize the main non-changable pointer (I2C full packet buffer):
		    pToStates->reassembledDataArray = i2cTxMasterBuffer;
		//5)Write rouine:
		 writeUsbToI2c(pToStates,  oneFrameBufferDev, oneFrameBufferHost);

		break;
	case read_from_i2c_dev:
		//read operation

		//1)Write data from one-frame buffer into a structure:
		  pToStates->typeOfAction = oneFrameBufferDev[0];
		  pToStates->numBytesToTransaction = (oneFrameBufferDev[2] << 8) | oneFrameBufferDev[1];
		  pToStates->slaveAddress = oneFrameBufferDev[3];
		//2)amount of transactions (roughly):
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction / 64;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction % 64) > 0) {
			pToStates->numUsbTransations++;
		}
		//4)Initialize the main non-changable pointer (I2C full packet buffer):
		    pToStates->reassembledDataArray = i2cRxMasterBuffer;
		    readUsbFromI2c(pToStates,  oneFrameBufferDev, oneFrameBufferHost);

		break;
	case read_last_stub_rx_i2c:
		//1)Write data from one-frame buffer into a structure:
				  pToStates->typeOfAction = oneFrameBufferDev[0];
				  pToStates->numBytesToTransaction = i2cSlaveHeader.i2cRxSlaveIndex;

				//2)amount of transactions (roughly):
				  pToStates->numUsbTransations = pToStates->numBytesToTransaction / 64;
				//3)When there is a remainder - add an extra transaction
				if ((pToStates->numBytesToTransaction % 64) > 0) {
					pToStates->numUsbTransations++;
				}
				//4)Initialize the main non-changable pointer (I2C full packet buffer):
				    pToStates->reassembledDataArray = i2cRxSlaveBuffer;
				    readUsbLastReadDataOfSlave(pToStates,  oneFrameBufferDev, oneFrameBufferHost);

	break;
	case  reset_interface:
		//reset I2C
		break;
	case write_tx_stub_buffer:
		//1)Write data from one-frame buffer into a structure:

		pToStates->typeOfAction = oneFrameBufferDev[0];
		pToStates->numBytesToTransaction = (oneFrameBufferDev[2] << 8) | oneFrameBufferDev[1];
		//2)amount of transactions (roughly):
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction / 64;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction % 64) > 0) {
			pToStates->numUsbTransations++;
		}
		//4)Initialize the main non-changable pointer (I2C full packet buffer):
			pToStates->reassembledDataArray = i2cTxSlaveBuffer;
		//5)
			 writeUsbSlaveTxBuffer(pToStates,  oneFrameBufferDev, oneFrameBufferHost);
		break;
	case setup_interface_i2c:
		//reset interfaces (a slave and a master) and setup parameters: slave address and speed of both
		memcpy(&temp32_1, oneFrameBufferDev + 4, 4);
		memcpy(&temp8_1, oneFrameBufferDev + 8, 1);
		temp8_1 <<= 1;
		resetI2cSetSpeedSlave(&i2cSlaveHeader, &I2C_InitStructSlave, temp32_1, temp8_1);
		resetI2cSetSpeedMaster(&i2cMasterHeader, &I2C_InitStructMaster, temp32_1);
		oneFrameBufferHost[0] = 0;
		sendReportToHost(oneFrameBufferHost);
		break;
	default:
		//write error code in report
		oneFrameBufferHost[0] = adapter_other_error;
		sendReportToHost(oneFrameBufferHost);
		 greenLedOff();
		return -2;
	}
	pToStates->typeOfAction = 0;
	 greenLedOff();
	return 0;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   /*

███╗░░░███╗░█████╗░██╗███╗░░██╗
████╗░████║██╔══██╗██║████╗░██║
██╔████╔██║███████║██║██╔██╗██║
██║╚██╔╝██║██╔══██║██║██║╚████║
██║░╚═╝░██║██║░░██║██║██║░╚███║
╚═╝░░░░░╚═╝╚═╝░░╚═╝╚═╝╚═╝░░╚══╝
     */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */



  I2C_InitStructMaster.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStructMaster.ClockSpeed = 100000;
  I2C_InitStructMaster.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStructMaster.OwnAddress1 = 0;
  I2C_InitStructMaster.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStructMaster.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;


  I2C_InitStructSlave.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStructSlave.ClockSpeed = 100000;
  I2C_InitStructSlave.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStructSlave.OwnAddress1 = 64;
  I2C_InitStructSlave.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStructSlave.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

  /*3) Initialize an instance of a structure,
   these values will be used in initialization and inside interrupts
   to restore options in DMA after transactions*/
      i2cSlaveHeader.hDevice = I2C1;  //device
      i2cSlaveHeader.i2cRxSlaveBuffer = i2cRxSlaveBuffer;  //Rx bufer
      i2cSlaveHeader.i2cTxSlaveBuffer = i2cTxSlaveBuffer; //Tx bufer
      i2cSlaveHeader.i2cSlaveRxPacketLength = 4;
      i2cSlaveHeader.i2cSlaveTxPacketLength = 4;
      i2cSlaveHeader.slaveI2cBufferLimit = 255;  //maximum length (not used)
      i2cSlaveHeader.i2cSlaveAddress = 0x40;  //must be shifted - because bit 0 - R/W
      i2cSlaveHeader.dmaRxChanel = DMA1_Channel7;
      i2cSlaveHeader.dmaTxChanel = DMA1_Channel6;
      //master initialization
	  i2cMasterHeader.dmaRxChanel = 0;
	  i2cMasterHeader.dmaTxChanel = 0;
	  i2cMasterHeader.hDevice = I2C2;
	  i2cMasterHeader.i2cMasterAddress = 0x20;
	  i2cMasterHeader.i2cMasterRxPacketLength = 16; //
	  i2cMasterHeader.i2cMasterTxPacketLength = 16;
	  i2cMasterHeader.i2cRxMasterBuffer = i2cRxMasterBuffer;
	  i2cMasterHeader.i2cTxMasterBuffer =  i2cTxMasterBuffer;
	  i2cMasterHeader.masterI2cBufferLimit = 1024;
	  i2cMasterHeader.dmaRxChanel = 0;
	  i2cMasterHeader.dmaTxChanel = 0;
	  i2cMasterHeader.WriteFlag = 1;



      //5) Init DMA and slave I2C ind Rx and Tx modes:
      i2c_usr_slave_init(&i2cSlaveHeader);
      //6)When there is the BUSY I2C bug, clear I2C and init again:
        if (I2C1->SR2 & I2C_SR2_BUSY) {
      	  //when busy bug, clean it by reset:
        	resetI2cSetSpeedSlave(&i2cSlaveHeader, &I2C_InitStructSlave, 100000, 64);
         }
	//init a master (RCC, GPIO initialization must be done before)
			i2c_usr_master_init(&i2cMasterHeader);
			if (I2C2->SR2 & I2C_SR2_BUSY) {
				//when the busy bug, clean it by reset:
				resetI2cSetSpeedMaster(&i2cMasterHeader, &I2C_InitStructMaster, 100000);
			}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			/*

			█░█░█ █░█ █ █░░ █▀▀
			▀▄▀▄▀ █▀█ █ █▄▄ ██▄
			 * */

  while (1)
  {

	  //readI2cAndWaitEnd(&i2cMasterHeader,32, i2cRxMasterBuffer, 32);


	 // 1)Set a semaphore
	   adapterSemaphoreUSB |= semaphore_await_rx_usb;
        while (adapterSemaphoreUSB & semaphore_await_rx_usb) {
    	//await a new data from a host
     }

    commandDispatcher(&i2cAdapterStates,  usb_buffer_for_report_host, usb_buffer_for_report_dev);





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 64;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* I2C2 interrupt Init */
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(I2C2_ER_IRQn);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13|LL_GPIO_PIN_14);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */






/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
