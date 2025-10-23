#include "main.h"
//Author: Andrii Androsovych
//Developed in the Code::Blocks
/**
The DLL library for STM32F103C8 (the blue pill) to use it as USB->CDC
The device is working in CDC COM mode
█░░ █ █▀▄▀█ █ ▀█▀ ▄▀█ ▀█▀ █ █▀█ █▄░█ █▀ ░   █▀▀ █▀█ █▄░█ █▀ ▀█▀ █▀█ ▄▀█ █ █▄░█ ▀█▀ █▀
█▄▄ █ █░▀░█ █ ░█░ █▀█ ░█░ █ █▄█ █░▀█ ▄█ █   █▄▄ █▄█ █░▀█ ▄█ ░█░ █▀▄ █▀█ █ █░▀█ ░█░ ▄█
maximum data length of the Master (I2C2) = 1024bytes
maximum data Length of the Slave = 256bytes (when more - wrap around)
*/
 // a sample exported function (only eample not used in app)
void DLL_EXPORT SomeFunction(const LPCSTR sometext)
{
    MessageBoxA(0, sometext, "DLL Message", MB_OK | MB_ICONINFORMATION);
}
 /** open CDC USB device (communication port)
  @hSerial - a pointer to handle, that will be initialized when port will be opened.
  @pathToPort - a path to port , for example: "\\\\.\\COM3"
 **/

int DLL_EXPORT openDevice (HANDLE* hSerial, LPCSTR pathToPort) {
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
        // 1) Open the COM port
    *hSerial = CreateFileA(
        pathToPort,  /* path to the port , for example:  "\\\\.\\COM3"  */
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (*hSerial == INVALID_HANDLE_VALUE) {
        return GetLastError();
    }
     // 2) Configure serial port
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    //read state of the COM port
    if (!GetCommState(*hSerial, &dcbSerialParams)) {
        return   GetLastError();
    }
    //set COM port pararmeters
    dcbSerialParams.BaudRate = CBR_115200; // ignored by CDC
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if (!SetCommState(*hSerial, &dcbSerialParams)) {
        return  GetLastError();
    }

    // 3) Configure timeouts - no timeouts , blocking mode
    timeouts.ReadIntervalTimeout         = 0;
    timeouts.ReadTotalTimeoutConstant    = 0;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.WriteTotalTimeoutConstant   = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(*hSerial, &timeouts)) {
         return GetLastError();
    }
    return 0;

}
 //close CDC USB device (COM port)
int DLL_EXPORT clodeDevice (HANDLE* hSerial) {
  CloseHandle(*hSerial);
  return 0;
}
/*
Reset both the slave i2c1 and the master i2c2 interfaces.Set speed for both devices.
Set the slave address for the slave device and allows acknowledge.
    @ states - a pointer to a strucure with type statesHandle
    @ pHidHandle - a handle to opened CDC virtual COM port,
    @ speedBps - speed in bPs
    @ slaveAddress - address of the slave i2c1 that will has been assigned
*/
int DLL_EXPORT resetI2cSetBothSpeedAndSlaveAddr(statesHandle * states,HANDLE* pHidHandle,unsigned int speedBps,unsigned char slaveAddress) {
    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesWritten;
    reportBufferHost[0] = setup_interface_i2c;
    memcpy(reportBufferHost+4, &speedBps,4);
    memcpy(reportBufferHost+8, &slaveAddress,1);

      if (!WriteFile(*pHidHandle, reportBufferHost, 64, &bytesWritten, NULL)) {

           return GetLastError();
    }
       if (!ReadFile(*pHidHandle, reportBufferDev, 64, &bytesWritten, NULL)) {

              return GetLastError();
       }
       return reportBufferHost[0];


}

/**
 Reads data through the master i2c2.
  @ states - a pointer to state structure with type 'statesHandle',
  @ pHidHandle - a handle to opened virtual CDC COM port
  @ slaveAddress - an address of remote slave device, from whom the data be read
  @ buffer - a storage with data, where the data be stored,
  @ amountOfData - amount of data to be received (bytes)

**/

short DLL_EXPORT readPacketFromUsbI2cAdapter(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char slaveAddress,
                              unsigned char* buffer,
                              unsigned short amountOfData) {

    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;
    //1)amount of transactions

    states->numBytesToTransaction = amountOfData;
    states->numUsbTransations = amountOfData / 64;
    if ( (states->numBytesToTransaction % 64) > 0 ) {
         states->numUsbTransations++;
    }
    //2)init pointers and address and operation
    states->typeOfAction = read_from_i2c_dev;
    states->slaveAddress = slaveAddress;
    states->buffPtr = buffer;
    states->reassembledDataArray  = buffer;
    //3)Send a read I2C request to a device:
    reportBufferDev[0] = states->typeOfAction; //operation code
    reportBufferDev[1] = (states->numBytesToTransaction & 0xff); //low byte
    reportBufferDev[2] = (states->numBytesToTransaction >> 8); //high byte
    reportBufferDev[3] = slaveAddress;   //slave address
    if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {
          // sprintf(messageBuffer, "Write failed with error %lu\n", GetLastError());
          // MessageBoxA(NULL, messageBuffer, "Notification", MB_OK | MB_ICONINFORMATION);
           return GetLastError();
    }
    //4)Wait a result of I2C read operation from a device:
    if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

              return GetLastError();
       }//Is an I2C transaction successfull?
    if (reportBufferHost[0] != 0) {
    //when a read operation on I2C device fail - return error code
             return reportBufferHost[0];
       }


     while (states->numUsbTransations > 1) {
          //5)Response to a device with 'read_from_i2c_dev'
          reportBufferDev[0] =  data_to_host;
        if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {

              return GetLastError();
           }
        //a)read a chunks from a device cosequently::
        if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

              return GetLastError();
           }
           //b.1)copying a packet of data into the buffer:
           memcpy(states->buffPtr, reportBufferHost, 64);
             //c)Increase the pointer:
           states->buffPtr += 64;
             //d)Decrease number of USB packets:
           states->numUsbTransations--;
             //e)..and number bytes for I2C transmission:
           states->numBytesToTransaction -= 64;
    }
    //when only one byte remaind to transmit:
     //7)send the 'read_from_i2c_dev' to the USB device:
            reportBufferDev[0] =  data_to_host;
    if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {

           return GetLastError();
       }
    //8)read the last piece of data from slave:
     if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

              return GetLastError();
        }
    //b.1)copying the last packet of data into the buffer:
           memcpy(states->buffPtr, reportBufferHost, states->numBytesToTransaction);
           return 0;

}


/**
 Sends data through master i2c2.
  @ states - a pointer to state structure with type 'statesHandle',
  @ pHidHandle - a handle to opened virtual CDC COM port
  @ slaveAddress - an address of remote slave device, whom the data be sent
  @ buffer - a storage with data, what will be rtansmitted,
  @ amountOfData - amount of data to be transmitted (bytes)

**/
short DLL_EXPORT sendPacketToUsbI2cAdapter(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char slaveAddress,
                              unsigned char* buffer,
                              unsigned short amountOfData) {


    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;



    //1)amount of transactions
    states->numBytesToTransaction = amountOfData;
    states->numUsbTransations = amountOfData / 64;
    if ( (states->numBytesToTransaction % 64) > 0 ) {
         states->numUsbTransations++;
    }
    //2)init pointers and address and operation
    states->slaveAddress = slaveAddress;
    states->typeOfAction = write_to_i2c_dev;
    states->buffPtr = buffer;
    states->reassembledDataArray  = buffer;
    //3)Command report:

    reportBufferDev[0] = states->typeOfAction; //operation code
    reportBufferDev[1] = (states->numBytesToTransaction & 0xff); //low byte
    reportBufferDev[2] = (states->numBytesToTransaction >> 8); //high byte
    reportBufferDev[3] = slaveAddress;   //slave address
    //4)Send data to the USB device (it is a blocking operation)
      if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {

           return GetLastError();
      }
    //5)iterate until the last USB transactiion:
    while (states->numUsbTransations > 1) {
        //a)await a signal "data_from_host" from a device:
        if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

              return GetLastError();
           } if (reportBufferHost[0] != data_from_host) {
               //("protocol error!"/
               return -1;
           }

         //b)Sends 64 byte packet to a device:
           //b.1)copying into the report_buffer:
           memcpy(reportBufferDev,states->buffPtr,64);
           //b.3)send the buffer to the USB device:
         if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {
           return GetLastError();
          }
         //c)Increase the pointer:
         states->buffPtr += 64;
         //d)Decrease number of USB packets:
         states->numUsbTransations--;
         //e)..and number bytes for I2C transmission:
         states->numBytesToTransaction -= 64;
    }
    //6)When only one USB transaction left:
    //await for device`s response:
      if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {
        return GetLastError();
    } if (reportBufferHost[0] != data_from_host) {
       //("protocol error!"/
       return -1;
    }
    //7)Decrease number of transactions (becomes 0):
    states->numUsbTransations--;
    //8)Send to device the last USB transaction:
      //8.1)copying into the report_buffer:
       memcpy(reportBufferDev,states->buffPtr,states->numBytesToTransaction);
       //8.3)send the buffer to the USB device:
     if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {
        return GetLastError();
     }
    //9)Now the device starts I2C transmission of data, that has been received later
    //So, the host awaits for response with result:
     if (!ReadFile (*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {
      return GetLastError();
    }
    //10)return received result:

    return reportBufferHost[0];

}
/**
  Read a packet of data, that had been received by slave i2c1 device.
  @ states - a pointer to structure with states,
  @ pHidHandle a handle to opened CDC COM port,
  @ buffer - a pointer when the data be stored,
  @ amountOfData - a pointer to uint16T variable, to write amount
    of received by slave i2c1 data (bytes).This amount a host doesn`t knows, so
    it must be transmitted from a device.

**/

short DLL_EXPORT readLastSlaveI2cReceivedPacket (statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short* amountOfData) {

    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;
    unsigned short temp16;

        //1)init pointers and address and operation
    states->typeOfAction = read_last_stub_rx_i2c;
    states->buffPtr = buffer;
    states->reassembledDataArray  = buffer;
    //2)Send request (read last received data i2c1) to a device:
    reportBufferDev[0] = states->typeOfAction; //operation code

    if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {
          // sprintf(messageBuffer, "Write failed with error %lu\n", GetLastError());
          // MessageBoxA(NULL, messageBuffer, "Notification", MB_OK | MB_ICONINFORMATION);
           return GetLastError();
    }
    //3) await and read status and total amount of last received (by a device) data:

        if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

          return GetLastError();
       }

    //Is the device not busy?
    if (reportBufferHost[0] != 0) {
    //when a read operation on I2C device fail - return error code
             return reportBufferHost[0];
    }
    //Read amount of data
     temp16 = ((unsigned short)reportBufferHost[2] << 8)|(unsigned short)reportBufferHost[1];

     //4)assign value to structure
     states->buffPtr = buffer;
     states->reassembledDataArray  = buffer;
     states->numBytesToTransaction = temp16;
     states->numUsbTransations = temp16 / 64;

     if ((temp16 % 64) > 0 ){
        states->numUsbTransations++;
     }

     *amountOfData = temp16;

     //5) Require packets (<=64 bytes) consequently:

     while (states->numUsbTransations > 1) {
          //5)Response to a device with 'read_from_i2c_dev'
          reportBufferDev[0] =  data_to_host;
        if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {

              return GetLastError();
           }
        //a)read a chunks from a device cosequently:
        if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

              return GetLastError();
           }
           //b.1)copying a packet of data into the buffer:
           memcpy(states->buffPtr, reportBufferHost, 64);
             //c)Increase the pointer:
           states->buffPtr += 64;
             //d)Decrease number of USB packets:
           states->numUsbTransations--;
             //e)..and number bytes for I2C transmission:
           states->numBytesToTransaction -= 64;
    }

        //when only one byte remaind to transmit:
     //7)send the 'read_from_i2c_dev' to the USB device:
            reportBufferDev[0] =  data_to_host;
    if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {

           return GetLastError();
       }
    //8)read the last piece of data from slave:
     if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

              return GetLastError();
        }
    //b.1)copying the last packet of data into the buffer:
           memcpy(states->buffPtr, reportBufferHost, states->numBytesToTransaction);
           return 0;

    }
/**
  Write data into slave transmitter i2c1 buffer.Wen a master requires
  data from a slave - these data will be sent.
   @states - a pointer to a state structure,
   @pHidHandle - a handle to opened CDC virtual COM port,
   @buffer - a storage ,from it data will be sent,
   @amountOfData - amount of data to be sent (bytes)

*/


short DLL_EXPORT writeSlaveI2cTransmitterBuffer(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData){
                                  ///


    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;
    //0) Shrink data:
    amountOfData &= 0x00ff;


    //1)amount of transactions
    states->numBytesToTransaction = amountOfData;
    states->numUsbTransations = amountOfData / 64;
    if ( (states->numBytesToTransaction % 64) > 0 ) {
         states->numUsbTransations++;
    }
    //2)init pointers operation
    states->typeOfAction =  write_tx_stub_buffer_i2c;
    states->buffPtr = buffer;
    states->reassembledDataArray  = buffer;
    //3)Command report:

    reportBufferDev[0] = states->typeOfAction; //operation code
    reportBufferDev[1] = (states->numBytesToTransaction & 0xff); //low byte (amount of data)
    reportBufferDev[2] = (states->numBytesToTransaction >> 8); //high byte (amount of data)

    //4)Send data to the USB device (it is a blocking operation)
      if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {

           return GetLastError();
      }

    //5)iterate until the last USB transactiion:
    while (states->numUsbTransations > 1) {
        //a)await a signal "data_from_host" from a device:
        if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

              return GetLastError();
           } if (reportBufferHost[0] != data_from_host) {
               //("protocol error!"/
               return -1;
           }

         //b)Sends 64 byte packet to a device:
           //b.1)copying into the report_buffer:
           memcpy(reportBufferDev,states->buffPtr,64);
           //b.3)send the buffer to the USB device:
         if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {
           return GetLastError();
          }
         //c)Increase the pointer:
         states->buffPtr += 64;
         //d)Decrease number of USB packets:
         states->numUsbTransations--;
         //e)..and number bytes for I2C transmission:
         states->numBytesToTransaction -= 64;
    }
    //6)When only one USB transaction left:
    //await for device`s response:
      if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {
        return GetLastError();
    } if (reportBufferHost[0] != data_from_host) {
       //("protocol error!"/
       return -1;
    }
    //7)Decrease number of transactions (becomes 0):
    states->numUsbTransations--;
    //8)Send to device the last USB transaction:
      //8.1)copying into the report_buffer:
       memcpy(reportBufferDev,states->buffPtr,states->numBytesToTransaction);
       //8.3)send the buffer to the USB device:
     if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {
        return GetLastError();
     }
     return 0;

}



extern "C" DLL_EXPORT BOOL APIENTRY DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
    switch (fdwReason)
    {
        case DLL_PROCESS_ATTACH:
            // attach to process
            // return FALSE to fail DLL load
            break;

        case DLL_PROCESS_DETACH:
            // detach from process
            break;

        case DLL_THREAD_ATTACH:
            // attach to thread
            break;

        case DLL_THREAD_DETACH:
            // detach from thread
            break;
    }
    return TRUE; // succesful
}

