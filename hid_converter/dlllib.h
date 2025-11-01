#ifndef DLLLIB_H_INCLUDED
#define DLLLIB_H_INCLUDED

#ifndef MYLIB_DYN_H
#define MYLIB_DYN_H
#include <windows.h>
#include "windows.h"
#include "stdio.h"
///defines for state mashine
#define MAX_REPORT_SIZE 64

///---------data markers-------------
#define  data_from_host   18
#define data_to_host  19
//-----------commands----------------
#define write_to_i2c_dev  24
#define read_from_i2c_dev 25
#define reset_interface   26
#define setup_interface_i2c   27
#define read_last_stub_rx_i2c 28
#define write_tx_stub_buffer_i2c  29
#define write_to_spi_dev  30
#define read_from_spi_dev 31
#define full_duplex_spi_dev  32
#define setup_spi_dev  33
//---error codes-------------------
#define adapter_success 0
#define adapter_AF 1
#define adapter_BERR 2
#define adapter_ARLO 3
#define adapter_OVR 4
#define adapter_timeout 5
#define adapter_other_error 6
#define adapter_busy 7
///payload offset for best CPU performance [report_id|0|0|0|payload_begin_here|....]
#define PAYLOAD_OFFS 4

//---defines of for LL library, used in STM32
 #define LL_SPI_MODE_MASTER 260
 #define  LL_SPI_MODE_SLAVE 0
 #define LL_SPI_PHASE_1EDGE 0
 #define LL_SPI_PHASE_2EDGE 1
 #define LL_SPI_POLARITY_LOW 0

 #define LL_SPI_POLARITY_HIGH 2
 #define LL_SPI_BAUDRATEPRESCALER_DIV2 0
 #define LL_SPI_BAUDRATEPRESCALER_DIV4 8
 #define  LL_SPI_BAUDRATEPRESCALER_DIV16 24
 #define  LL_SPI_BAUDRATEPRESCALER_DIV32 32
 #define  LL_SPI_BAUDRATEPRESCALER_DIV64 40
 #define  LL_SPI_BAUDRATEPRESCALER_DIV128 48
 #define  LL_SPI_BAUDRATEPRESCALER_DIV256 56
 #define LL_SPI_LSB_FIRST 128
 #define LL_SPI_MSB_FIRST 0
//Author: Andrii Androsovych

typedef struct {
		unsigned char typeOfAction;
		unsigned char* buffPtr; //changes during exec.
		unsigned short numUsbTransations;
		unsigned short numBytesToTransaction;
		unsigned char*  reassembledDataArray; //not changed
		unsigned char slaveAddress;
} statesHandle;


// function pointer typedefs
typedef void  (*pSomeFunction)(const LPCSTR sometext);
typedef int   (*pOpenDevice)(HANDLE* hSerial, LPCSTR pathToPort);
typedef int   (*pCloseDevice)(HANDLE* hSerial);
typedef int   (*pResetI2cSetBothSpeedAndSlaveAddr)(statesHandle* states,
                                                   HANDLE* pHidHandle,
                                                   unsigned int speedBps,
                                                   unsigned char slaveAddress);

typedef short (*pReadPacketFromUsbI2cAdapter)(statesHandle* states,
                                              HANDLE* pHidHandle,
                                              unsigned char slaveAddress,
                                              unsigned char* buffer,
                                              unsigned short amountOfData);

typedef short (*pSendPacketToUsbI2cAdapter)(statesHandle* states,
                                            HANDLE* pHidHandle,
                                            unsigned char slaveAddress,
                                            unsigned char* buffer,
                                            unsigned short amountOfData);

typedef short (*pReadLastSlaveI2cReceivedPacket) (statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short* amountOfData);

typedef short (*pWriteSlaveI2cTransmitterBuffer)(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData);

typedef short (*pSendPacketToUsbSpiAdapter )(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData  );

typedef short (*pReadPacketFromUsbSpiAdapter )(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData  );

typedef short (*pFullDuplexSpiTransaction )( statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* txBuffer,
                              unsigned char* rxBuffer,
                              unsigned short amountOfData );

typedef short (* pSetupSpi)( statesHandle * states,HANDLE* pHidHandle,
                            unsigned short clkPolarity, unsigned short clkPhase,
                            unsigned short baudRate, unsigned short lsbFirst,
                            unsigned short ssHighPol );




#endif

#endif // DLLLIB_H_INCLUDED
