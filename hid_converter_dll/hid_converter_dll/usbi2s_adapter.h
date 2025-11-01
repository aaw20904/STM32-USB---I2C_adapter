#ifndef USBI2S_ADAPTER_H_INCLUDED
#define USBI2S_ADAPTER_H_INCLUDED

#ifndef __MAIN_H__
#define __MAIN_H__

#include <windows.h>

/*  To use this exported function of dll, include this header
 *  in your project.
 */

#ifdef BUILD_DLL
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __declspec(dllimport)
#endif


#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
		unsigned char typeOfAction;
		unsigned char* buffPtr; //changes during exec.
		unsigned short numUsbTransations;
		unsigned short numBytesToTransaction;
		unsigned char*  reassembledDataArray; //not changed
		unsigned char slaveAddress;
} statesHandle;

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

void DLL_EXPORT SomeFunction(const LPCSTR sometext);

int DLL_EXPORT openDevice (HANDLE* hSerial, LPCSTR pathToPort);

int DLL_EXPORT closeDevice (HANDLE* hSerial);

int DLL_EXPORT resetI2cSetBothSpeedAndSlaveAddr(statesHandle * states, HANDLE* pHidHandle,
                                                unsigned int speedBps, unsigned char slaveAddress);

short DLL_EXPORT readPacketFromUsbI2cAdapter(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char slaveAddress,
                              unsigned char* buffer,
                              unsigned short amountOfData);

short DLL_EXPORT sendPacketToUsbI2cAdapter(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char slaveAddress,
                              unsigned char* buffer,
                              unsigned short amountOfData);

short DLL_EXPORT writeSlaveI2cTransmitterBuffer(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData);

short DLL_EXPORT readLastSlaveI2cReceivedPacket (statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short* amountOfData);

 short DLL_EXPORT sendPacketToUsbSpiAdapter(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData);

short DLL_EXPORT readPacketFromUsbSpiAdapter(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData);

short DLL_EXPORT fullDuplexSpiTransaction(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* txBuffer,
                              unsigned char* rxBuffer,
                              unsigned short amountOfData);

short DLL_EXPORT setupSpi(statesHandle * states, HANDLE* pHidHandle,
            unsigned short clkPolarity, unsigned short clkPhase,
            unsigned short baudRate, unsigned short lsbFirst,
            unsigned short ssHighPol);


#ifdef __cplusplus
}
#endif

#endif // __MAIN_H__


#endif // USBI2S_ADAPTER_H_INCLUDED
