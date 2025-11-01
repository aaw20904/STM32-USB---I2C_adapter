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



///---------data markers-------------
#define  data_from_host   18
#define data_to_host  19
//-----------commands----------------
#define write_to_slave  24
#define read_from_slave 25
#define reset_interface 26
#define setup_interface 27
//---error codes-------------------
#define adapter_success 0
#define adapter_AF 1
#define adapter_BERR 2
#define adapter_ARLO 3
#define adapter_OVR 4
#define adapter_timeout 5
#define adapter_other_error 6
#define adapter_busy 7

void DLL_EXPORT SomeFunction(const LPCSTR sometext);

int DLL_EXPORT openDevice (HANDLE* hSerial, LPCSTR pathToPort);

int DLL_EXPORT clodeDevice (HANDLE* hSerial);

int DLL_EXPORT resetI2cSetBothSpeedAndSlaveAddr(HANDLE* pHidHandle,unsigned int speedBps,unsigned char slaveAddress);

short DLL_EXPORT readPacketFromUsbI2cAdapter(
                              HANDLE* pHidHandle,
                              unsigned char slaveAddress,
                              unsigned char* buffer,
                              unsigned short amountOfData);

short DLL_EXPORT sendPacketToUsbI2cAdapter(
                              HANDLE* pHidHandle,
                              unsigned char slaveAddress,
                              unsigned char* buffer,
                              unsigned short amountOfData);



#ifdef __cplusplus
}
#endif

#endif // __MAIN_H__
