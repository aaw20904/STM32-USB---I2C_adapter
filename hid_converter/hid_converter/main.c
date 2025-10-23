#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <hidsdi.h>
#include <setupapi.h>
#include "adapter.h"
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



    pSomeFunction someFunction;
    pOpenDevice openDevice;
    pCloseDevice closeDevice;
    pResetI2cSetBothSpeedAndSlaveAddr resetI2cSetBothSpeedAndSlaveAddr;
    pReadPacketFromUsbI2cAdapter readPacketFromUsbI2cAdapter;
    pSendPacketToUsbI2cAdapter sendPacketToUsbI2cAdapter;
    pReadLastSlaveI2cReceivedPacket readLastSlaveI2cReceivedPacket;
    pWriteSlaveI2cTransmitterBuffer writeSlaveI2cTransmitterBuffer;



unsigned char usbInBuffer[516];
HMODULE hMyLib = NULL;




int main (void) {
    unsigned short slaveDeviceDataCount;


    HANDLE hSerial =0;

    hMyLib = LoadLibraryA("hid_converter_dll.dll");
    if (!hMyLib) {
        printf("Failed to load DLL, error %lu\n", GetLastError());
        return -1;
    }
    // resolve function addresses
    someFunction  = (pSomeFunction) GetProcAddress(hMyLib, "SomeFunction");
    openDevice    = (pOpenDevice)   GetProcAddress(hMyLib, "openDevice");
    closeDevice   = (pCloseDevice)  GetProcAddress(hMyLib, "clodeDevice"); // note: typo?
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


     if (!someFunction || !openDevice || !closeDevice ||
        !resetI2cSetBothSpeedAndSlaveAddr ||
        !readPacketFromUsbI2cAdapter || !sendPacketToUsbI2cAdapter ||
        !readLastSlaveI2cReceivedPacket || !writeSlaveI2cTransmitterBuffer) {
        printf("Failed to resolve one or more functions, error %lu\n", GetLastError());
        FreeLibrary(hMyLib);
        hMyLib = NULL;
        return -2;
    }


  strcpy(usbOutBuffer,	"So she was considering in her own mind (as well as she could, for the "
		"hot day made her feel very sleepy and stupid), whether the pleasure of "
		"making a daisy-chain would be worth the trouble of getting up and "
		"picking the daisies, when suddenly a White Rabbit, Buks Bunny, he eating a carrot /0");

       if (openDevice(&hSerial,"\\\\.\\COM5" ) != 0){
         printf("COM port open fail %d",GetLastError());
         return -1;
       }
        // Write 64 bytes
       printf("Device opened successfully!\n");

   resetI2cSetBothSpeedAndSlaveAddr(&mashineHandle, &hSerial,10000, 40);
   printf("Write status: %d \n", sendPacketToUsbI2cAdapter(&mashineHandle, &hSerial,40,usbOutBuffer,0x0064));
      writeSlaveI2cTransmitterBuffer(&mashineHandle,&hSerial,textString,0x0064);
   printf("read status: %d \n", readPacketFromUsbI2cAdapter(&mashineHandle, &hSerial,40,usbInBuffer,0x0064));
    printf("%s \n", usbInBuffer);
      readLastSlaveI2cReceivedPacket(&mashineHandle,&hSerial,usbInBuffer,&slaveDeviceDataCount);
      printf("%s \n", usbInBuffer);
        ///at the end - close COM port
        CloseHandle(hSerial);


    return 0;
}


/*

int OldMain(void) {



    HDEVINFO hDevInfo = SetupDiGetClassDevs(&hidGuid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        printf("Error getting device info\n");
        return 1;
    }

    SP_DEVICE_INTERFACE_DATA devInterfaceData;
    devInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &hidGuid, i, &devInterfaceData); i++) {
        DWORD requiredSize = 0;
        SetupDiGetDeviceInterfaceDetail(hDevInfo, &devInterfaceData, NULL, 0, &requiredSize, NULL);
        PSP_DEVICE_INTERFACE_DETAIL_DATA devDetail = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(requiredSize);
        devDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

        if (SetupDiGetDeviceInterfaceDetail(hDevInfo, &devInterfaceData, devDetail, requiredSize, NULL, NULL)) {
            HANDLE hDevice = CreateFile(devDetail->DevicePath,
                                        GENERIC_READ | GENERIC_WRITE,
                                        FILE_SHARE_READ | FILE_SHARE_WRITE,
                                        NULL,
                                        OPEN_EXISTING,
                                        FILE_FLAG_OVERLAPPED, // Use 0 for blocking
                                        NULL);
            if (hDevice != INVALID_HANDLE_VALUE) {
                // Optional: check VID/PID
                HIDD_ATTRIBUTES attrib;
                attrib.Size = sizeof(HIDD_ATTRIBUTES);
                HidD_GetAttributes(hDevice, &attrib);
                if (attrib.VendorID == VENDOR_ID && attrib.ProductID == PRODUCT_ID) {
                    printf("Device opened\n");

                    // --- Send report ---
                    unsigned char outReport[MAX_REPORT_SIZE] = {0};
                    outReport[0] = 0x01; // Report ID
                    outReport[1] = 0xAA; // Example data
                    if (!HidD_SetOutputReport(hDevice, outReport, MAX_REPORT_SIZE)) {
                        printf("Write failed\n");
                    }

                    // --- Read report ---
                    unsigned char inReport[MAX_REPORT_SIZE] = {0};
                    if (HidD_GetInputReport(hDevice, inReport, MAX_REPORT_SIZE)) {
                        printf("Read: %02X\n", inReport[1]);
                    }

                    CloseHandle(hDevice);
                }
            }
        }
        free(devDetail);
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    return 0;

}
*/
