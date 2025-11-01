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
/**

█▀▄ █▀█   ▀█▀ █░█ █ █▀   ▄▀█ █▀▀ ▀█▀ █▀▀ █▀█   █▀▀ ▄▀█ █▀▀ █░█   █▀▀ █░█ █▄▄ █▀▀
█▄▀ █▄█   ░█░ █▀█ █ ▄█   █▀█ █▀░ ░█░ ██▄ █▀▄   ██▄ █▀█ █▄▄ █▀█   █▄▄ █▄█ █▄█ ██▄

█▀█ █▀▀ ▄▄ █▀▀ █▀▀ █▄░█ █▀▀ █▀█ ▄▀█ ▀█▀ █ █▀█ █▄░█   █ █▄░█   █░█ █▀ █▄▄ █▀▄ ▄▄ █▀▀ █▀▄ █▀▀ ░ █▀▀ ▀
█▀▄ ██▄ ░░ █▄█ ██▄ █░▀█ ██▄ █▀▄ █▀█ ░█░ █ █▄█ █░▀█   █ █░▀█   █▄█ ▄█ █▄█ █▄▀ ░░ █▄▄ █▄▀ █▄▄ ▄ █▄▄ ▄
 * */
/* H E L P:
   To bind  "USB transmission complete" callback
   in HAL USB driver, the following steps recommended:
   1) There are __weak void CDC_TxCpltCallback(void) in usbd_cdc_if.c file.
   2)Insert into middlewares/ST/..../src/usbd_cdc.c the following declaration:

	extern void CDC_TxCpltCallback (void);

   3)Find the transmission callback static uint8_t  USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
     and insert before    "return USBD_OK;" call of external (implemented im main.c) function

          CDC_TxCpltCallback ();

   4) Define callback function in main.c:

    void CDC_TxCpltCallback(void) {
       adapterSemaphoreUSB &= ~semaphore_await_tx_usb;
    }

 * */
//Author: Andrii Androsovych
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
maximum data length of the Master (SPI1) = 1024bytes
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
volatile wrp_spi_master_header spiMasterHeader;

  unsigned char usb_buffer_for_report_dev[68];
  unsigned char usb_buffer_for_report_host[68];
//volatile unsigned char usb_buffer_for_report[68];

  volatile uint8_t i2cRxSlaveBuffer[260];
  volatile uint8_t i2cTxSlaveBuffer[260]= { "Alice was beginning to get very tired of sitting by her sister on the bank, and of having nothing to do: once or twice she had peeped into the book her sister was reading, but it had no pictures or conversations in it, 'and what is the use of a book,'.."};

  volatile uint8_t i2cRxMasterBuffer[1026]={0};
  volatile uint8_t i2cTxMasterBuffer[1026]={0};

  volatile uint8_t spiRxMasterBuffer[1026]={0};
  volatile uint8_t spiTxMasterBuffer[1026]={0};


LL_I2C_InitTypeDef I2C_InitStructSlave = {0};

LL_I2C_InitTypeDef I2C_InitStructMaster = {0};




//USB semaphore
volatile uint32_t adapterSemaphoreUSB;
//I2C semaphore
volatile uint32_t adapterSemaphoreI2C;

volatile uint32_t adapterSemaphoreSPI;

 statesHandle i2cAdapterStates;

 void awaitReportFromHost (void);
 void sendReportToHost( uint8_t* oneFrameBuffer);

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
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */


//a****callback***for*****USB**********CDC***
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len) {
	// Copy data from USB internal buffer
	memcpy (usb_buffer_for_report_dev, Buf, Len);
	 //clear a semaphore
	adapterSemaphoreUSB &= ~semaphore_await_rx_usb;

}

void CDC_TxCpltCallback (void) {
    adapterSemaphoreUSB &= ~semaphore_await_tx_usb;
}



void dummyDelay(uint32_t delay){

	while(delay>0){
		delay--;
	}
}
///// SPI procedures
/***
 *

░██████╗██████╗░██╗
██╔════╝██╔══██╗██║
╚█████╗░██████╔╝██║
░╚═══██╗██╔═══╝░██║
██████╔╝██║░░░░░██║
╚═════╝░╚═╝░░░░░╚═╝
 * **/
 void setSlaveSelectSPI(uint8_t pol) {

		 if (pol==0) {
			 GPIOA->BSRR = GPIO_BSRR_BR4;

		 } else {
			 GPIOA->BSRR = GPIO_BSRR_BS4;
		 }

 }


 void clearSlaveSelectSPI(uint8_t pol) {
	 if (pol==0) {
			 GPIOA->BSRR = GPIO_BSRR_BS4;

		 } else {
			 GPIOA->BSRR = GPIO_BSRR_BR4;
		 }
 }
  ///NOTE: The receive and transmit amount of data are EQUAL and THE SAME!
 //all the paameters in according to LL library constants
 void customSpiFullduplexMasterSetup(uint16_t clkPol, uint16_t clkPh, uint16_t baudRate, uint16_t msbLsb)
 {
     LL_SPI_InitTypeDef SPI_InitStruct = {0};

     // Ensure peripheral clock enabled
     //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
     //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

     // Disable SPI before reconfig
     LL_SPI_Disable(SPI1);

     // Basic SPI configuration
     SPI_InitStruct.TransferDirection =LL_SPI_FULL_DUPLEX;
     SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
     SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
     SPI_InitStruct.ClockPolarity = clkPol; // LL_SPI_POLARITY_LOW
     SPI_InitStruct.ClockPhase = clkPh;// LL_SPI_PHASE_1EDGE
     SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;   // <<< important change
     SPI_InitStruct.BaudRate = baudRate; // LL_SPI_BAUDRATEPRESCALER_DIV64
     SPI_InitStruct.BitOrder = msbLsb; // LL_SPI_MSB_FIRST
     SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
     SPI_InitStruct.CRCPoly = 10;

     LL_SPI_Init(SPI1, &SPI_InitStruct);

     // Enable SPI peripheral
    // LL_SPI_Enable(SPI1);
 }







//full duplex function , works well, tested.MOSI - output, MISO - input.During communication?
// Tx read byte for transmission from Tx buffer, and received byte write into Rx bufer.So both buffers are used.
int32_t masterFullduplexSpiTransactionDMA(volatile wrp_spi_master_header* spiHeader){
	adapterSemaphoreSPI &= 0xfffffffe; //clear bit 0
	adapterSemaphoreSPI |= 0x00000001;
	spiHeader->spiMasterErrorFlags = 0;
	//-------------------------------------------
	//a) disable SPI
	spiHeader->hDevice->CR1 &= ~SPI_CR1_SPE;
	//enable DMA on Tx
	spiHeader->hDevice->CR2 |= SPI_CR2_TXDMAEN;
	//enable DMA on Rx
	spiHeader->hDevice->CR2 |= SPI_CR2_RXDMAEN;
	   //c)disable interrupts on Tx
	spiHeader->hDevice->CR2 &=~(SPI_CR2_TXEIE);
	   //disable interrupts on Rx
	spiHeader->hDevice->CR2 &= ~SPI_CR2_RXNEIE;
     //d)Disable DMA channel (Tx)
    spiHeader->dmaTxChanel->CCR &= ~DMA_CCR_EN;
	 //e)Set number data to transmitt
    spiHeader->dmaTxChanel->CNDTR = spiHeader->spiMasterTxPacketLength-1;
	//f)Set memory address (CMAR)
    spiHeader->dmaTxChanel->CMAR = (uint32_t)spiHeader->spiTxMasterBuffer+1;
    //g)peripherial addrss
    spiHeader->dmaTxChanel->CPAR = (uint32_t)&SPI1->DR;
    //enable DMA interrupt Transmission Complete
    spiHeader->dmaTxChanel->CCR |= DMA_CCR_TCIE;
    //--rx dma
    //d)Disable DMA channel (Rx)
   	spiHeader->dmaRxChanel->CCR &= ~DMA_CCR_EN;
   	 //e)Set number data to transmitt
   	spiHeader->dmaRxChanel->CNDTR = spiHeader->spiMasterRxPacketLength;
   	//f)Set memory address (CMAR)
   	spiHeader->dmaRxChanel->CMAR = (uint32_t)spiHeader->spiRxMasterBuffer;
   	//g)peripherial addrss
   	spiHeader->dmaRxChanel->CPAR = (uint32_t)&SPI1->DR;
   	//enable DMA interrupt Transmission Complete
   	spiHeader->dmaRxChanel->CCR |= DMA_CCR_TCIE;
    //write the first byte manually
   	spiHeader->hDevice->DR = spiHeader->spiTxMasterBuffer[0];
	 //enable DMA channels
   	if (spiHeader->dmaTxChanel->CNDTR > 0) {
   		spiHeader->dmaTxChanel->CCR |= DMA_CCR_EN;
   	}

	 spiHeader->dmaRxChanel->CCR |= DMA_CCR_EN;
	 //g)set SS signal in active state
	 setSlaveSelectSPI(spiHeader->ssPolarity);
	 //dummyDelay(10);

	  //c)enable peripherial - start SPI transaction
	 spiHeader->hDevice->CR1 |= SPI_CR1_SPE;

    while (adapterSemaphoreSPI & 0x00000001) {
     //wait until sends txPacketLength-1 bytes
    }

    while (spiHeader->hDevice->SR & SPI_SR_BSY) {
    //wait until last byte has been sent
    }

    //dummyDelay(10);
    //when all the data has been transmitted - turn the peripherial off
    	 //spiHeader->hDevice->CR1 &= ~SPI_CR1_SPE;
         SPI1->CR1 &= ~SPI_CR1_SPE;
         //d)Disable DMA channel (Tx)
        spiHeader->dmaTxChanel->CCR &= ~DMA_CCR_EN;
    	spiHeader->dmaRxChanel->CCR &= ~DMA_CCR_EN;
    //clear SS signal to inactive
	 clearSlaveSelectSPI(spiHeader->ssPolarity);

		//clear TXE flag
	 spiHeader->hDevice->DR = 0x00;
	  //disable DMA
	 spiHeader->hDevice->CR2 &= ~SPI_CR2_TXDMAEN;
	 spiHeader->hDevice->CR2 &= ~SPI_CR2_RXDMAEN;
	  //
	return 0;
}




//for half-duplex, so buggy




int32_t writeUsbToSpi (statesHandle* pToStates,
		  uint8_t* oneFrameBufferDev,
		  uint8_t* oneFrameBufferHost) {

		  unsigned char temp=0;
		  unsigned char* tempCharPtr =0;
	pToStates->typeOfAction = data_from_host;
	//1)Copy data into SPI struture
	spiMasterHeader.spiMasterRxPacketLength = pToStates->numBytesToTransaction;
	spiMasterHeader.spiMasterTxPacketLength = pToStates->numBytesToTransaction;
	spiMasterHeader.spiTxMasterBuffer = pToStates->reassembledDataArray;
	spiMasterHeader.spiRxMasterBuffer = spiRxMasterBuffer;

	/*3)Initialize the pointer (it was changed in previous transaction  */
	tempCharPtr = (void*)pToStates->reassembledDataArray;
	pToStates->buffPtr = tempCharPtr;
	/*4)Iterate, until str.numUsbTransations == 1 */
	while (pToStates->numUsbTransations > 1) {
		/*4.1)Send to a host request 'data_from_host':*/
		oneFrameBufferHost[0] = data_from_host;
		sendReportToHost(oneFrameBufferHost);

		/*4.2)await new message from the host (data will be stored in the 'oneFrameBuffer'):*/
		awaitReportFromHost();
		  //4.3)copy 64 bytes into SPI buffer
		memcpy((void*)pToStates->buffPtr, oneFrameBufferDev, 64);
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
	memcpy((void*)pToStates->buffPtr, oneFrameBufferDev , pToStates->numBytesToTransaction);
    //9)Restore value amount of bytes  for a transaction:
	pToStates->numBytesToTransaction = pToStates->buffPtr - pToStates->reassembledDataArray;

	//10)Start SPI transaction:
	temp = masterFullduplexSpiTransactionDMA(&spiMasterHeader);
	//11)Send result of operation to the host:
	oneFrameBufferHost[0]= temp;
	sendReportToHost( oneFrameBufferHost);
    pToStates->typeOfAction = 0;
    return 0;
}

int32_t readUsbFromSpi (statesHandle* pToStates,
						  uint8_t* oneFrameBufferDev,
						  uint8_t* oneFrameBufferHost) {

	unsigned char temp=0;
	unsigned char* tempCharPtr =0;
	pToStates->typeOfAction = data_to_host;
	//prepare a buffer
	spiMasterHeader.spiRxMasterBuffer=pToStates->reassembledDataArray;
	spiMasterHeader.spiTxMasterBuffer = spiTxMasterBuffer;
	spiMasterHeader.spiMasterRxPacketLength = pToStates->numBytesToTransaction;
	spiMasterHeader.spiMasterTxPacketLength = pToStates->numBytesToTransaction;
	//clear Tx buffer (optionally not neccessary)
	memset(spiMasterHeader.spiTxMasterBuffer,0x00,512);
	//1)Read data from SPI:
	temp = masterFullduplexSpiTransactionDMA(&spiMasterHeader);
	//2)Send returned status  to a Host:
	oneFrameBufferHost[0]  = temp;
	sendReportToHost(oneFrameBufferHost);

	//3)When the transaction fail - return
	if (temp != 0) {
		return -1;
	}
	//4)When a transaction successfull - wait a request 'data_to_host' from a host

	/*3)Initialize the pointer (it was changed in previous transaction  */
	tempCharPtr = (void*)pToStates->reassembledDataArray;
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
			memcpy(oneFrameBufferHost, (void*)pToStates->buffPtr, 64);
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
			memcpy(oneFrameBufferHost ,(void*)pToStates->buffPtr,  pToStates->numBytesToTransaction);
			 //9)Response to the host with the last  data packet:
			sendReportToHost( oneFrameBufferHost);
           return 0;

}

int32_t fullDuplexSpiUsbTransaction (statesHandle* pToStates,
		volatile wrp_spi_master_header* spiHeader,
		  uint8_t* oneFrameBufferDev,
		  uint8_t* oneFrameBufferHost) {
	//for safety:
				if ( pToStates->numBytesToTransaction >= spiHeader->masterSpiBufferLimit){
					pToStates->numBytesToTransaction >= spiHeader->masterSpiBufferLimit;
				}
		 //uint16_t amountOfDataToProcessing =  pToStates->numBytesToTransaction;
		  unsigned char temp=0;
		  unsigned char* tempCharPtr =0;
		  uint16_t  lvNumUsbTrans, lvNumBytesToSent;
		  //save locally USB transaction count and size of a  packet :
		  lvNumUsbTrans = pToStates->numUsbTransations;
		  lvNumBytesToSent = pToStates->numBytesToTransaction;

		//10)Copy data into SPI struture
		spiHeader->spiMasterRxPacketLength = pToStates->numBytesToTransaction;
		spiHeader->spiMasterTxPacketLength = pToStates->numBytesToTransaction;
		spiHeader->spiTxMasterBuffer = pToStates->reassembledDataArray;
		spiHeader->spiRxMasterBuffer = spiRxMasterBuffer;
        pToStates->typeOfAction = data_from_host;

	/*3)Initialize the pointer (it was changed in previous transaction  */
	tempCharPtr = (void*)pToStates->reassembledDataArray;
	pToStates->buffPtr = tempCharPtr;
	/*4)Iterate, until str.numUsbTransations == 1 */
	while (pToStates->numUsbTransations > 1) {
		/*4.1)Send to a host request 'data_from_host':*/
		oneFrameBufferHost[0] = data_from_host;
		sendReportToHost(oneFrameBufferHost);

		/*4.2)await new message from the host (data will be stored in the 'oneFrameBuffer'):*/
		awaitReportFromHost();
		  //4.3)copy 64 bytes into SPI buffer
		memcpy((void*)pToStates->buffPtr, oneFrameBufferDev, 64);
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
	memcpy((void*)pToStates->buffPtr, oneFrameBufferDev , pToStates->numBytesToTransaction);
    //9)Restore value amount of bytes  for a transaction:
	pToStates->numBytesToTransaction = pToStates->buffPtr - pToStates->reassembledDataArray;

	//10)Start SPI transaction:
	temp = masterFullduplexSpiTransactionDMA(&spiMasterHeader);


	//11)Restore USB transaction parameters, later saved:
	  pToStates->numUsbTransations =  lvNumUsbTrans;
	   pToStates->numBytesToTransaction = lvNumBytesToSent;
	   //assign Rx buffr as main array for transaction it to a host:
	   pToStates->reassembledDataArray = spiRxMasterBuffer;

	   /*12)Initialize the pointer (it was changed in previous transaction  */
	   	tempCharPtr = (void*)pToStates->reassembledDataArray;
	   	pToStates->buffPtr = tempCharPtr;
	   	//13)Send result of operation to the host:
	   		oneFrameBufferHost[0]= temp;
	   		sendReportToHost( oneFrameBufferHost);
	   	//13.1)when action was fail - return error code
	   		if(temp != 0){
	   			return temp;
	   		}
	   	//14)Send chunks to a host consequently:
	   	/*Iterate, until str.numUsbTransations == 1 */
	   		while (pToStates->numUsbTransations > 1) {
	   			awaitReportFromHost();
	   			if (oneFrameBufferDev[0] != data_to_host)
	   			   {
	   					//when incorrect request from a host - return error
	   					oneFrameBufferHost[0] = adapter_other_error;
	   					return -1;
	   			   }
	   			  //14.1)copy 64 bytes from I2C buffer
	   			memcpy(oneFrameBufferHost, (void*)pToStates->buffPtr, 64);
	   			pToStates->buffPtr += 64;
	   			  //14.3)decrease number of transactions:
	   			pToStates->numUsbTransations--;
	   			  //14.4)decrease amount of bytes:
	   			pToStates->numBytesToTransaction -= 64;
	   			  //14.5)Response to the host with a data packet
	   			sendReportToHost( oneFrameBufferHost);


	   		}
	   		//there is only one piece of data
	   		 //15) Waiting new data from a host
	   		awaitReportFromHost();
	   				if (oneFrameBufferDev[0] != data_to_host) {
	   						//when incorrect request from a host - return error
	   						oneFrameBufferHost[0] = adapter_other_error;
	   						return -1;
	   				}
	   		  /*16) copy last data piece into the buffer */
	   			memcpy(oneFrameBufferHost ,(void*)pToStates->buffPtr,  pToStates->numBytesToTransaction);
	   			 //17)Response to the host with the last  data packet:
	   			sendReportToHost( oneFrameBufferHost);

    pToStates->typeOfAction = 0;
    return 0;
}





/*

██╗██████╗░░█████╗░
██║╚════██╗██╔══██╗
██║░░███╔═╝██║░░╚═╝
██║██╔══╝░░██║░░██╗
██║███████╗╚█████╔╝
╚═╝╚══════╝░╚════╝░
 * */

/// I2C procedures

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


uint8_t writeI2cAndWaitEnd (volatile wrp_i2c_master_header* i2cHeader, unsigned char address,  volatile unsigned char* bufferPtr, unsigned short amountOfData) {
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

void sendReportToHost( uint8_t* oneFrameBuffer){
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
	temp = readI2cAndWaitEnd(&i2cMasterHeader, pToStates->slaveAddress, (void*)pToStates->reassembledDataArray, pToStates->numBytesToTransaction);
	//2)Send returned status  to a Host:
	oneFrameBufferHost[0]  = temp;
	sendReportToHost(oneFrameBufferHost);

	//3)When the transaction fail - return
	if (temp != 0) {
		return -1;
	}
	//4)When a transaction successfull - wait a request 'data_to_host' from a host

	/*3)Initialize the pointer (it was changed in previous transaction  */
	tempCharPtr = (void*)pToStates->reassembledDataArray;
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
			memcpy(oneFrameBufferHost, (void*)pToStates->buffPtr, 64);
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
			memcpy(oneFrameBufferHost ,(void*)pToStates->buffPtr,  pToStates->numBytesToTransaction);
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
		tempCharPtr = (void*) pToStates->reassembledDataArray;
		pToStates->buffPtr = tempCharPtr;
		/*4)Iterate, until str.numUsbTransations == 1 */
		while (pToStates->numUsbTransations > 1) {
			/*4.1)Send to a host request 'data_from_host':*/
			oneFrameBufferHost[0] = data_from_host;
			sendReportToHost(oneFrameBufferHost);

			/*4.2)await new message from the host (data will be stored in the 'oneFrameBuffer'):*/
			awaitReportFromHost();
			  //4.3)copy 64 bytes into I2C buffer
			memcpy((void*)pToStates->buffPtr, oneFrameBufferDev, 64);
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
		memcpy((void*)pToStates->buffPtr, oneFrameBufferDev, pToStates->numBytesToTransaction);
       return 0;
}


int32_t readUsbLastReadDataOfSlave (statesHandle* pToStates,
								   uint8_t* oneFrameBufferDev,
								   uint8_t* oneFrameBufferHost) {

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
			memcpy(oneFrameBufferHost, (void*)pToStates->buffPtr, 64);
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
			memcpy(oneFrameBufferHost ,(void*)pToStates->buffPtr,  pToStates->numBytesToTransaction);
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
	tempCharPtr = (void*)pToStates->reassembledDataArray;
	pToStates->buffPtr = tempCharPtr;
	/*4)Iterate, until str.numUsbTransations == 1 */
	while (pToStates->numUsbTransations > 1) {
		/*4.1)Send to a host request 'data_from_host':*/
		oneFrameBufferHost[0] = data_from_host;
		sendReportToHost(oneFrameBufferHost);

		/*4.2)await new message from the host (data will be stored in the 'oneFrameBuffer'):*/
		awaitReportFromHost();
		  //4.3)copy 64 bytes into I2C buffer
		memcpy((void*)pToStates->buffPtr, oneFrameBufferDev, 64);
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
	memcpy((void*)pToStates->buffPtr, oneFrameBufferDev , pToStates->numBytesToTransaction);
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


/*

█▀▀ █▀█ █▀▄▀█ █▀▄▀█ ▄▀█ █▄░█ █▀▄   █▀█ █▀█ █░█ ▀█▀ █▀▀ █▀█
█▄▄ █▄█ █░▀░█ █░▀░█ █▀█ █░▀█ █▄▀   █▀▄ █▄█ █▄█ ░█░ ██▄ █▀▄
 * */

//----------------------------------------------------------------------------
int32_t commandRouter (statesHandle* pToStates,
						   uint8_t* oneFrameBufferHost, //data buffer for the Host (outgoing)
						   uint8_t* oneFrameBufferDev   //data buffer for device (incoming)
							) {
	uint32_t temp32_1;
	uint8_t temp8_1;
	uint16_t* ptr16;
	uint16_t spiPh, spiPol, spiLsbFirst,spiDrate,spiSsHigh;
	uint16_t delay;
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
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction >> 6;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction & 0x3f) > 0) {
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
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction >> 6;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction & 0x3f) > 0) {
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
				  pToStates->numUsbTransations = pToStates->numBytesToTransaction >> 6;
				//3)When there is a remainder - add an extra transaction
				if ((pToStates->numBytesToTransaction & 0x3f) > 0) {
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
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction >> 6;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction & 0x3f) > 0) {
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
		delay = 0;
		while(delay < 256){
			delay++;
		}
		oneFrameBufferHost[0] = 0;
		sendReportToHost(oneFrameBufferHost);
		break;
	case setup_spi_dev:
		//set-up the SPI1 interface:
		//a)extract data from a usb packet into variables:
		ptr16 = (void*)oneFrameBufferDev;
		spiPh = *(ptr16++);
		spiPol = *(ptr16++);
		spiDrate = *(ptr16++);
		spiLsbFirst = *(ptr16++);
		spiSsHigh = *(ptr16++);
		//b)set SS polarity
		spiMasterHeader.ssPolarity = (uint8_t)spiSsHigh;
		//c)and apply it immediately on the SS pin:
		clearSlaveSelectSPI((uint8_t)spiSsHigh);
        //d)set-up SPI
		customSpiFullduplexMasterSetup(spiPol,spiPh,spiDrate,spiLsbFirst);
		delay = 0;
		while (delay < 256) {
					delay++;
				}
		//e)response with success to a host
		oneFrameBufferHost[0] = 0;
		sendReportToHost(oneFrameBufferHost);
		break;
	case write_to_spi_dev:
		//write operation
		//1)Write data from one-frame buffer into a structure:

		  pToStates->typeOfAction = oneFrameBufferDev[0];
		  pToStates->numBytesToTransaction = (oneFrameBufferDev[2] << 8) | oneFrameBufferDev[1];
		//2)amount of transactions (roughly):
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction >> 6;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction & 0x3f) > 0) {
			pToStates->numUsbTransations++;
		}
		//4)Initialize the main non-changable pointer (I2C full packet buffer):
		    pToStates->reassembledDataArray = spiTxMasterBuffer;
		//5)Write rouine:
		    writeUsbToSpi(pToStates,  oneFrameBufferDev, oneFrameBufferHost);
		    delay = 0;
			while (delay < 256) {
						delay++;
					}
		break;
	case read_from_spi_dev:
		//read operation

		//1)Write data from one-frame buffer into a structure:
		  pToStates->typeOfAction = oneFrameBufferDev[0];
		  pToStates->numBytesToTransaction = (oneFrameBufferDev[2] << 8) | oneFrameBufferDev[1];
		//2)amount of transactions (roughly):
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction >> 6;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction & 0x3f) > 0) {
			pToStates->numUsbTransations++;
		}
		//4)Initialize the main non-changable pointer (I2C full packet buffer):
			pToStates->reassembledDataArray = spiRxMasterBuffer;
			readUsbFromSpi(pToStates,  oneFrameBufferDev, oneFrameBufferHost);
			delay = 0;
			while (delay < 256) {
						delay++;
					}
		break;
	case full_duplex_spi_dev:
		//1)Write data from one-frame buffer into a structure:

		  pToStates->typeOfAction = oneFrameBufferDev[0];
		  pToStates->numBytesToTransaction = (oneFrameBufferDev[2] << 8) | oneFrameBufferDev[1];
		//2)amount of transactions (roughly):
		  pToStates->numUsbTransations = pToStates->numBytesToTransaction >> 6;
		//3)When there is a remainder - add an extra transaction
		if ((pToStates->numBytesToTransaction & 0x3f) > 0) {
			pToStates->numUsbTransations++;
		}
		//4)Initialize the main non-changable pointer (I2C full packet buffer):
		    pToStates->reassembledDataArray = spiTxMasterBuffer;
		//5)Write rouine:
		    fullDuplexSpiUsbTransaction(pToStates,&spiMasterHeader,  oneFrameBufferDev, oneFrameBufferHost);
		    delay = 0;
			while (delay < 256) {
						delay++;
					}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
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
	  i2cMasterHeader.masterI2cBufferLimit = 1024;      ///Attention - master I2C buffer limit is here!
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



		spiMasterHeader.hDevice = SPI1;
		spiMasterHeader.spiMasterTxPacketLength = 16;            //ATTENTION:
		spiMasterHeader.masterSpiBufferLimit = 1024;             //SPI packet limit here!
		spiMasterHeader.spiRxMasterBuffer = spiRxMasterBuffer;
		spiMasterHeader.spiTxMasterBuffer = spiTxMasterBuffer;
		spiMasterHeader.ssPolarity = 0;
		spiMasterHeader.dmaRxChanel = DMA1_Channel2;

		spiMasterHeader.hDevice = SPI1;
		spiMasterHeader.spiMasterRxPacketLength = 16;
		spiMasterHeader.masterSpiBufferLimit = 1024;          //SPI packet limit here!
		spiMasterHeader.spiRxMasterBuffer = spiRxMasterBuffer;
		spiMasterHeader.spiTxMasterBuffer = spiTxMasterBuffer;
		spiMasterHeader.ssPolarity = 0;
		spiMasterHeader.dmaTxChanel = DMA1_Channel3;



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

    commandRouter(&i2cAdapterStates,  usb_buffer_for_report_host, usb_buffer_for_report_dev);

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* SPI1_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13|LL_GPIO_PIN_14);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
