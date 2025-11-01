#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <hidsdi.h>
#include <setupapi.h>

#include "dlllib.h"
///for STM32 board , defined in Cube Code Generator`s window
#define VENDOR_ID  1155
#define PRODUCT_ID 22352
#define REPORT_ID 1

#define CP2112_VENDOR_ID 0x10c4
#define CP2112_PRODUCT_ID 0xea90
//Author: Andrii Androsovych



WORD cp2112Version;
BYTE cp2112PartNumber;


DWORD bytesProcessedG;
statesHandle mashineHandle;



unsigned char usbOutBuffer[512];
unsigned char  test[512];
unsigned char textString[] = {"The rabbit-hole went straight on like a tunnel for some way"
", and then dipped suddenly down, so suddenly"
" that Alice had not a moment to think about stopping herself before "
"she found herself falling down a very deep well."};

  /*Declare pointers to functions*/

    pSomeFunction someFunction;
    pOpenDevice openDevice;
    pCloseDevice closeDevice;
    pResetI2cSetBothSpeedAndSlaveAddr resetI2cSetBothSpeedAndSlaveAddr;
    pReadPacketFromUsbI2cAdapter readPacketFromUsbI2cAdapter;
    pSendPacketToUsbI2cAdapter sendPacketToUsbI2cAdapter;
    pReadLastSlaveI2cReceivedPacket readLastSlaveI2cReceivedPacket;
    pWriteSlaveI2cTransmitterBuffer writeSlaveI2cTransmitterBuffer;
    pSetupSpi setupSpi;
    pSendPacketToUsbSpiAdapter sendPacketToUsbSpiAdapter;
    pReadPacketFromUsbSpiAdapter readPacketFromUsbSpiAdapter;
    pFullDuplexSpiTransaction  fullDuplexSpiTransaction;

unsigned char usbInBuffer[516];
HMODULE hMyLib = NULL;


WORD cp2112Version;
BYTE cp2112PartNumber;


DWORD bytesProcessedG;
statesHandle mashineHandle;

unsigned char usbInBuffer[516];


int main (void) {
    unsigned short slaveDeviceDataCount;


    HANDLE hSerial =0;
   //Open dll file
   hMyLib = LoadLibraryA("hid_converter_dll.dll");
    if (!hMyLib) {
        printf("Failed to load DLL, error %lu\n", GetLastError());
        return -1;
    }
    // resolve function addresses - assign to the function pointers
    someFunction  = (pSomeFunction) GetProcAddress(hMyLib, "SomeFunction");
    openDevice    = (pOpenDevice)   GetProcAddress(hMyLib, "openDevice");
    closeDevice   = (pCloseDevice)  GetProcAddress(hMyLib, "closeDevice"); // note: typo?
    resetI2cSetBothSpeedAndSlaveAddr =
                  (pResetI2cSetBothSpeedAndSlaveAddr)
                  GetProcAddress(hMyLib, "resetI2cSetBothSpeedAndSlaveAddr");
    readPacketFromUsbI2cAdapter =
                  (pReadPacketFromUsbI2cAdapter)
                  GetProcAddress(hMyLib, "readPacketFromUsbI2cAdapter");
    sendPacketToUsbI2cAdapter =
                  (pSendPacketToUsbI2cAdapter)
                  GetProcAddress(hMyLib, "sendPacketToUsbI2cAdapter");
     readLastSlaveI2cReceivedPacket =
                  (pReadLastSlaveI2cReceivedPacket)
                  GetProcAddress(hMyLib, "readLastSlaveI2cReceivedPacket");
     writeSlaveI2cTransmitterBuffer =
                  (pWriteSlaveI2cTransmitterBuffer)
                  GetProcAddress(hMyLib, "writeSlaveI2cTransmitterBuffer");
     setupSpi = (pSetupSpi)
                  GetProcAddress(hMyLib,"setupSpi");
      sendPacketToUsbSpiAdapter = ( pSendPacketToUsbSpiAdapter )
                GetProcAddress(hMyLib,"sendPacketToUsbSpiAdapter");
      readPacketFromUsbSpiAdapter       = ( pReadPacketFromUsbSpiAdapter )
                GetProcAddress(hMyLib,"readPacketFromUsbSpiAdapter");
      fullDuplexSpiTransaction      = ( pFullDuplexSpiTransaction )
                GetProcAddress(hMyLib,"fullDuplexSpiTransaction");

     /*Check - are all the procedures opened successfully?*/

     if (!someFunction || !openDevice || !closeDevice || !sendPacketToUsbSpiAdapter ||
        !resetI2cSetBothSpeedAndSlaveAddr || !readPacketFromUsbSpiAdapter ||
        !readPacketFromUsbI2cAdapter || !sendPacketToUsbI2cAdapter ||  !fullDuplexSpiTransaction ||
        !readLastSlaveI2cReceivedPacket || !writeSlaveI2cTransmitterBuffer|| !setupSpi) {
        printf("Failed to resolve one or more functions, error %lu\n", GetLastError());
        FreeLibrary(hMyLib);
        hMyLib = NULL;
        return -2;
    }



  strcpy((void*)usbOutBuffer,	"So she was considering in her own mind (as well as she could, for the "
		"hot day made her feel very sleepy and stupid), whether the pleasure of "
		"making a daisy-chain would be worth the trouble of getting up and "
		"picking the daisies, when suddenly a White Rabbit, Buks Bunny, he eating a carrot /0");

       if (openDevice(&hSerial,"\\\\.\\COM3" ) != 0){
         printf("COM port open fail %d",GetLastError());
         return -1;
       }
   /*Example for SPI*/
       printf("Device opened successfully!\n");
       setupSpi(&mashineHandle,&hSerial,LL_SPI_POLARITY_HIGH,LL_SPI_PHASE_1EDGE,LL_SPI_BAUDRATEPRESCALER_DIV256,LL_SPI_MSB_FIRST,0);
        sendPacketToUsbSpiAdapter(&mashineHandle,&hSerial,textString,80);
        memset(test,0x00,256);
       readPacketFromUsbSpiAdapter(&mashineHandle,&hSerial,test,2);
        fullDuplexSpiTransaction(&mashineHandle,&hSerial,textString,test,100);

  /*Example for I2C*/
       resetI2cSetBothSpeedAndSlaveAddr(&mashineHandle, &hSerial,10000, 40);
       //send 100 bytes through master Tx to slave Rx
       printf("Write status: %d \n", sendPacketToUsbI2cAdapter(&mashineHandle, &hSerial,40,usbOutBuffer,0x0064));
          //write in slave Tx buffer (about hole rabbit)
          writeSlaveI2cTransmitterBuffer(&mashineHandle,&hSerial,textString,0x0064);
          //read data by master Rx, from slave Tx
       printf("read status: %d \n", readPacketFromUsbI2cAdapter(&mashineHandle, &hSerial,40,usbInBuffer,0x0064));
       printf("%s \n", usbInBuffer);
         //read data from slave Rx buffer
      readLastSlaveI2cReceivedPacket(&mashineHandle,&hSerial,usbInBuffer,&slaveDeviceDataCount);
      printf("%s \n", usbInBuffer);
        ///at the end - close COM port



        CloseHandle(hSerial);


    return 0;
}


