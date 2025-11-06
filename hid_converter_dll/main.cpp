#include "usbi2s_adapter.h"

/**
The DLL library for STM32F103C8 (the blue pill) to use it as USB->CDC
The device is working in CDC COM mode
█░░ █ █▀▄▀█ █ ▀█▀ ▄▀█ ▀█▀ █ █▀█ █▄░█ █▀ ░   █▀▀ █▀█ █▄░█ █▀ ▀█▀ █▀█ ▄▀█ █ █▄░█ ▀█▀ █▀
█▄▄ █ █░▀░█ █ ░█░ █▀█ ░█░ █ █▄█ █░▀█ ▄█ █   █▄▄ █▄█ █░▀█ ▄█ ░█░ █▀▄ █▀█ █ █░▀█ ░█░ ▄█
maximum data length of the Master (I2C2) = 1024bytes
maximum data Length of the Slave = 256bytes (when more - wrap around)
maximum SPI data length - 1024 bytes
*/
 // a sample exported function
void DLL_EXPORT SomeFunction(const LPCSTR sometext)
{
    MessageBoxA(0, sometext, "DLL Message", MB_OK | MB_ICONINFORMATION);
}


//The functions warppers for write/read 64 bytes in/from CDC  VCP
//WINBOOL means int
int writePacketToUsbDev(HANDLE* usbHandle, unsigned char* dataToWrite, unsigned long* bytesWritten) {
  return (int)WriteFile(*usbHandle,  dataToWrite, 64, (LPDWORD)bytesWritten, NULL);
};

int readPacketFromUsbDev(HANDLE* usbHandle, unsigned char* dataToRead,  unsigned long* bytesRead) {
  return (int)ReadFile(*usbHandle, dataToRead, 64, (LPDWORD)bytesRead, NULL);
}

/*
H A R D W A R E   N O T E:
The board - STM32F103 , well known as "blue pill".The I2C2 works as master device.
The I2C1 works as slave device.
The SPI1 always works as a master in full-duplex mode.
*/
 /**Open a virtual com port (VCP).Through this port
 the library communicates with USB to I@C, SPI adapter  */
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
int DLL_EXPORT closeDevice (HANDLE* hSerial) {
  CloseHandle(*hSerial);
  return 0;
}

/*
This function setups parameters of I2C2 (Master) and I2C1 (Slave) interfaces.
@ statesHandle * states - a pointer to a "statesHandle" structure
@HANDLE* pHidHandle - a handle to opened Virtual Com Port
@unsigned int speedBps - speed of interface in bits per second
@unsigned char slaveAddress - slave address of the I2C1
*/

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
    unsigned long bytesWritten;
    reportBufferHost[0] = setup_interface_i2c;
    memcpy(reportBufferHost+4, &speedBps,4);
    memcpy(reportBufferHost+8, &slaveAddress,1);
     //WriteFile(*pHidHandle, reportBufferHost, 64, &bytesWritten, NULL)
     //ReadFile(*pHidHandle, reportBufferDev, 64, &bytesWritten, NULL)
      if (! writePacketToUsbDev(pHidHandle, reportBufferHost, &bytesWritten)) {

           return GetLastError();
      }
       if (! readPacketFromUsbDev(pHidHandle, reportBufferDev, &bytesWritten)) {

              return GetLastError();
       }
       return reportBufferHost[0];


}


///function for reading data through I2C2 (master)
/*
@statesHandle * states - a pointer to state structure;
@HANDLE* pHidHandle - a pointer to VCP opened port;
@unsigned char slaveAddress - an address of slae device, where data be read;
@unsigned char* buffer -  where the data be stored;
@unsigned short amountOfData - amount of bytes to be read by I2C2 (Master)
*/
/**
 Reads data through the master i2c2.
  @ states - a pointer to state structure with type 'statesHandle',
  @ pHidHandle - a handle to opened virtual CDC COM port
  @ slaveAddress - an address of remote slave device, from whom the data be read
  @ buffer - a storage with data, where the data be stored,
  @ amountOfData - amount of data to be received (bytes)

**/

short DLL_EXPORT readPacketWithMasterI2c(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char slaveAddress,
                              unsigned char* buffer,
                              unsigned short amountOfData) {

    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;
    //1)amount of transactions

    states->numBytesToTransaction = amountOfData;
    states->numUsbTransations = amountOfData >> 6;   //amountOfData / 64
    if ( (states->numBytesToTransaction & 0x3F) > 0 ) {  //numBytesToTransaction % 64
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

    if (!writePacketToUsbDev(pHidHandle, reportBufferDev, &bytesProcessed)) {
          // sprintf(messageBuffer, "Write failed with error %lu\n", GetLastError());
          // MessageBoxA(NULL, messageBuffer, "Notification", MB_OK | MB_ICONINFORMATION);
           return GetLastError();
    }
    //4)Wait a result of I2C read operation from a device:

    if (! readPacketFromUsbDev(pHidHandle, reportBufferHost, &bytesProcessed)) {

              return GetLastError();
       }//Is an I2C transaction successfull?
    if (reportBufferHost[0] != 0) {
    //when a read operation on I2C device fail - return error code
             return reportBufferHost[0];
       }


     while (states->numUsbTransations > 1) {
          //5)Response to a device with 'read_from_i2c_dev'
          reportBufferDev[0] =  data_to_host;

        if (!writePacketToUsbDev(pHidHandle, reportBufferDev,   &bytesProcessed)) {

              return GetLastError();
           }
        //a)read a chunks from a device cosequently::

        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,   &bytesProcessed)) {

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

    if (!writePacketToUsbDev(pHidHandle, reportBufferDev, &bytesProcessed)) {

           return GetLastError();
       }
    //8)read the last piece of data from slave:
    //ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)

     if (!readPacketFromUsbDev(pHidHandle, reportBufferHost, &bytesProcessed)) {

              return GetLastError();
        }
    //b.1)copying the last packet of data into the buffer:
           memcpy(states->buffPtr, reportBufferHost, states->numBytesToTransaction);
           return 0;

}


///function for sending data through I2C2 (master)
/*
@statesHandle * states - a pointer to state structure;
@HANDLE* pHidHandle - a pointer to VCP opened port;
@unsigned char slaveAddress - an address of a slave device, where data be written;
@unsigned char* buffer -  from where the data be read;
@unsigned short amountOfData - amount of bytes to be write by I2C2 (Master)
*/
/**
 Sends data through master i2c2.
  @ states - a pointer to state structure with type 'statesHandle',
  @ pHidHandle - a handle to opened virtual CDC COM port
  @ slaveAddress - an address of remote slave device, whom the data be sent
  @ buffer - a storage with data, what will be rtansmitted,
  @ amountOfData - amount of data to be transmitted (bytes)

**/
short DLL_EXPORT writePacketWithMasterI2c(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char slaveAddress,
                              unsigned char* buffer,
                              unsigned short amountOfData) {


    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;

    //1)amount of transactions
    states->numBytesToTransaction = amountOfData;
    states->numUsbTransations = amountOfData >> 6;
    if ( (states->numBytesToTransaction & 0x3F) > 0 ) {
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
    //WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)

      if (! writePacketToUsbDev(pHidHandle, reportBufferDev,   &bytesProcessed)) {

           return GetLastError();
      }
    //5)iterate until the last USB transactiion:
    while (states->numUsbTransations > 1) {
        //a)await a signal "data_from_host" from a device:
        //ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)
        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost, &bytesProcessed)) {

              return GetLastError();
           } if (reportBufferHost[0] != data_from_host) {
               //("protocol error!"/
               return -1;
           }

         //b)Sends 64 byte packet to a device:
           //b.1)copying into the report_buffer:
           memcpy(reportBufferDev,states->buffPtr,64);
           //b.3)send the buffer to the USB device:
           //WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)
         if (!writePacketToUsbDev(pHidHandle, reportBufferDev,  &bytesProcessed)) {
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
    //ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)

    if (!readPacketFromUsbDev(pHidHandle, reportBufferHost, &bytesProcessed)) {
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
       //WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)

     if (!writePacketToUsbDev(pHidHandle, reportBufferDev,  &bytesProcessed)) {
        return GetLastError();
     }
    //9)Now the device starts I2C transmission of data, that has been received later
    //So, the host awaits for response with result:
    //ReadFile (*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)
     if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,  &bytesProcessed)) {
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
   //WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)
    if (!writePacketToUsbDev(pHidHandle, reportBufferDev,  &bytesProcessed)) {
          // sprintf(messageBuffer, "Write failed with error %lu\n", GetLastError());
          // MessageBoxA(NULL, messageBuffer, "Notification", MB_OK | MB_ICONINFORMATION);
           return GetLastError();
    }
    //3) await and read status and total amount of last received (by a device) data:
      //ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)
        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost, &bytesProcessed)) {

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
     states->numUsbTransations = temp16 >> 6;  /// divided by 64

     if ((temp16 % 64) > 0 ){
        states->numUsbTransations++;
     }

     *amountOfData = temp16;

     //5) Require packets (<=64 bytes) consequently:

     while (states->numUsbTransations > 1) {
          //5)Response to a device with 'read_from_i2c_dev'
          reportBufferDev[0] =  data_to_host;
          //WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)
        if (!writePacketToUsbDev(pHidHandle, reportBufferDev, &bytesProcessed)) {

              return GetLastError();
           }
        //a)read a chunks from a device cosequently:
        //ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)
        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost, &bytesProcessed)) {

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
            //WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)
    if (!writePacketToUsbDev(pHidHandle, reportBufferDev,  &bytesProcessed)) {

           return GetLastError();
       }
    //8)read the last piece of data from slave:
    //ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)
     if (!readPacketFromUsbDev(pHidHandle, reportBufferHost, &bytesProcessed)) {

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
    states->numUsbTransations = amountOfData >> 6;
    if ( (states->numBytesToTransaction & 0x3F) > 0 ) {
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
      if (!writePacketToUsbDev(pHidHandle, reportBufferDev,   &bytesProcessed )) {

           return GetLastError();
      }

    //5)iterate until the last USB transactiion:
    while (states->numUsbTransations > 1) {
        //a)await a signal "data_from_host" from a device:
        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,   &bytesProcessed )) {

              return GetLastError();
           } if (reportBufferHost[0] != data_from_host) {
               //("protocol error!"/
               return -1;
           }

         //b)Sends 64 byte packet to a device:
           //b.1)copying into the report_buffer:
           memcpy(reportBufferDev,states->buffPtr,64);
           //b.3)send the buffer to the USB device:
         if (!writePacketToUsbDev(pHidHandle, reportBufferDev,   &bytesProcessed )) {
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
      if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,  &bytesProcessed )) {
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
     if (!writePacketToUsbDev(pHidHandle, reportBufferDev,  &bytesProcessed )) {
        return GetLastError();
     }
     return 0;

}


///the function to send data through the SPI1 interface.
//N O T E:  The SPI1 works ALWAYS in full duplex mode in this library
/*
  @statesHandle * states - a pointer to a states structure,
  @HANDLE* pHidHandle - a handle of opened VCP port,
  @unsigned char* buffer - from where the data be read ,
  @unsigned short amountOfData - amount of data to be sent

*/

  short DLL_EXPORT sendPacketToUsbSpiAdapter(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData) {


    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;



    //1)amount of transactions
    states->numBytesToTransaction = amountOfData;
    states->numUsbTransations = amountOfData >> 6;
    if ( (states->numBytesToTransaction & 0x3F) > 0 ) {
         states->numUsbTransations++;
    }
    //2)init pointers and address and operation
    states->typeOfAction = write_to_spi_dev;
    states->buffPtr = buffer;
    states->reassembledDataArray  = buffer;
    //3)Command report:

    reportBufferDev[0] = states->typeOfAction; //operation code
    reportBufferDev[1] = (states->numBytesToTransaction & 0xff); //low byte
    reportBufferDev[2] = (states->numBytesToTransaction >> 8); //high byte

    //4)Send data to the USB device (it is a blocking operation)
      if (!writePacketToUsbDev(pHidHandle, reportBufferDev, &bytesProcessed )) {

           return GetLastError();
      }
    //5)iterate until the last USB transactiion:
    while (states->numUsbTransations > 1) {
        //a)await a signal "data_from_host" from a device:
        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,   &bytesProcessed )) {

              return GetLastError();
           } if (reportBufferHost[0] != data_from_host) {
               //("protocol error!"/
               return -1;
           }

         //b)Sends 64 byte packet to a device:
           //b.1)copying into the report_buffer:
           memcpy(reportBufferDev,states->buffPtr,64);
           //b.3)send the buffer to the USB device:
         if (!writePacketToUsbDev(pHidHandle, reportBufferDev , &bytesProcessed )) {
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
      if (!readPacketFromUsbDev(pHidHandle, reportBufferHost , &bytesProcessed )) {
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
     if (!writePacketToUsbDev(pHidHandle, reportBufferDev,  &bytesProcessed )) {
        return GetLastError();
     }
    //9)Now the device starts I2C transmission of data, that has been received later
    //So, the host awaits for response with result:
     if (!readPacketFromUsbDev (pHidHandle, reportBufferHost,  &bytesProcessed )) {
      return GetLastError();
    }
    //10)return received result:

    return reportBufferHost[0];

}///the function to read data from SPI1
//N O T E:  The SPI1 works ALWAYS in full duplex mode in this library
/*
  @statesHandle * states - a pointer to a states structure,
  @HANDLE* pHidHandle - a handle of opened VCP port,
  @unsigned char* buffer -  where the data be written ,
  @unsigned short amountOfData - amount of data to be read

*/

short DLL_EXPORT readPacketFromUsbSpiAdapter(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* buffer,
                              unsigned short amountOfData) {

    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;
    //1)amount of transactions

    states->numBytesToTransaction = amountOfData;
    states->numUsbTransations = amountOfData >> 6;  // divided by 64
    if ( (states->numBytesToTransaction & 0x3F) > 0 ) {  // % 64
         states->numUsbTransations++;
    }
    //2)init pointers and address and operation
    states->typeOfAction = read_from_spi_dev;
    states->buffPtr = buffer;
    states->reassembledDataArray  = buffer;
    //3)Send a read I2C request to a device:
    reportBufferDev[0] = states->typeOfAction; //operation code
    reportBufferDev[1] = (states->numBytesToTransaction & 0xff); //low byte
    reportBufferDev[2] = (states->numBytesToTransaction >> 8); //high byte

    if (!writePacketToUsbDev(pHidHandle, reportBufferDev,   &bytesProcessed )) {
          // sprintf(messageBuffer, "Write failed with error %lu\n", GetLastError());
          // MessageBoxA(NULL, messageBuffer, "Notification", MB_OK | MB_ICONINFORMATION);
           return GetLastError();
    }
    //4)Wait a result of I2C read operation from a device:
    if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,  &bytesProcessed )) {

              return GetLastError();
       }//Is an I2C transaction successfull?
    if (reportBufferHost[0] != 0) {
    //when a read operation on I2C device fail - return error code
             return reportBufferHost[0];
       }


     while (states->numUsbTransations > 1) {
          //5)Response to a device with 'read_from_i2c_dev'
          reportBufferDev[0] =  data_to_host;
        if (!writePacketToUsbDev(pHidHandle, reportBufferDev,   &bytesProcessed )) {

              return GetLastError();
           }
        //a)read a chunks from a device cosequently::
        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,   &bytesProcessed )) {

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
    if (!writePacketToUsbDev(pHidHandle, reportBufferDev, &bytesProcessed )) {

           return GetLastError();
       }
    //8)read the last piece of data from slave:
     if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,   &bytesProcessed )) {

              return GetLastError();
        }
    //b.1)copying the last packet of data into the buffer:
           memcpy(states->buffPtr, reportBufferHost, states->numBytesToTransaction);
           return 0;

}
///the function to write/read simultaneously
//N O T E:  The SPI1 works ALWAYS in full duplex mode in this library
/*
  @statesHandle * states - a pointer to a states structure,
  @unsigned char* txBuffer - wrom where the data be read,
  @unsigned char* rxBuffer - where the data be written,
  @unsigned short amountOfData - amount of data of transaction

*/



 short DLL_EXPORT fullDuplexSpiTransaction(statesHandle * states,
                              HANDLE* pHidHandle,
                              unsigned char* txBuffer,
                              unsigned char* rxBuffer,
                              unsigned short amountOfData) {

    unsigned short lvNumUsbTrans, lvNumBytesToSent;
    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesProcessed;



    //1)amount of transactions
    states->numBytesToTransaction = amountOfData;
    states->numUsbTransations = amountOfData >> 6; ///  / 64;
    if ( (states->numBytesToTransaction & 0x3F ) > 0 ) { //% 64
         states->numUsbTransations++;
    }
    //1.1)store s.numBytesToTransaction, s.numUsbTransactions
    //into local variables   lvNumUsbTrans, lvNumBytesToSent
    lvNumUsbTrans = states->numUsbTransations;
    lvNumBytesToSent = states->numBytesToTransaction;
    //2)init pointers and address and operation
    states->typeOfAction = full_duplex_spi_dev ;
    states->buffPtr = txBuffer;
    states->reassembledDataArray  = txBuffer;
    //3)Command report:

    reportBufferDev[0] = states->typeOfAction; //operation code
    reportBufferDev[1] = (states->numBytesToTransaction & 0xff); //low byte
    reportBufferDev[2] = (states->numBytesToTransaction >> 8); //high byte

    //4)Send data to the USB device (it is a blocking operation)
      if (!writePacketToUsbDev(pHidHandle, reportBufferDev,   &bytesProcessed )) {

           return GetLastError();
      }
    //5)iterate until the last USB transactiion:
    while (states->numUsbTransations > 1) {
        //a)await a signal "data_from_host" from a device:
        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost, &bytesProcessed )) {

              return GetLastError();
           } if (reportBufferHost[0] != data_from_host) {
               //("protocol error!"/
               return -1;
           }

         //b)Sends 64 byte packet to a device:
           //b.1)copying into the report_buffer:
           memcpy(reportBufferDev,states->buffPtr,64);
           //b.3)send the buffer to the USB device:
         if (!writePacketToUsbDev(pHidHandle, reportBufferDev, &bytesProcessed )) {
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
      if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,   &bytesProcessed )) {
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
     if (!writePacketToUsbDev(pHidHandle, reportBufferDev,  &bytesProcessed )) {
        return GetLastError();
     }
    //9)Now the device starts I2C transmission of data, that has been received later
    //So, the host awaits for response with result:
     if (!readPacketFromUsbDev (pHidHandle, reportBufferHost,  &bytesProcessed )) {
      return GetLastError();
    }

    if(reportBufferHost[0] != 0){
            //when write operation was fail - return error code
        return reportBufferHost[0];
    }
    // now data are in rx buffer of device.
    //10)Restore USB transactions parametrs:
       states->numUsbTransations =  lvNumUsbTrans;
      states->numBytesToTransaction  =lvNumBytesToSent;
    //11) Assign rx buffer as main buffer - here will be wrote receivd data from a USB device:
      states->reassembledDataArray = rxBuffer;
      states->buffPtr = rxBuffer;
       while (states->numUsbTransations > 1) {
          //)Response to a device with a data request
          reportBufferDev[0] =  data_to_host;
        if (!writePacketToUsbDev(pHidHandle, reportBufferDev,   &bytesProcessed )) {

              return GetLastError();
           }
        //a)read a chunks from a device cosequently::
        if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,  &bytesProcessed )) {

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
     //12)send the 'read_from_i2c_dev' to the USB device:
            reportBufferDev[0] =  data_to_host;
    if (!writePacketToUsbDev(pHidHandle, reportBufferDev,  &bytesProcessed )) {

           return GetLastError();
       }
    //13)read the last piece of data from slave:
     if (!readPacketFromUsbDev(pHidHandle, reportBufferHost,   &bytesProcessed )) {

              return GetLastError();
        }
    //14)copying the last packet of data into the buffer:
           memcpy(states->buffPtr, reportBufferHost, states->numBytesToTransaction);
    return reportBufferHost[0];

}
//Setup parameters for SPI1 interface
/*
@ statesHandle * states - pointer to a structure,
@ HANDLE* pHidHandle - pointer  to opened VCP port
   (----next-----4--parameters---are---from--LL--STM32---library)
@ unsigned short clkPolarity - may be: LL_SPI_POLARITY_LOW ,LL_SPI_POLARITY_HIGH
@ unsigned short clkPhase - may be LL_SPI_PHASE_1EDGE, LL_SPI_PHASE_2EDGE
@ unsigned short baudRate - the prescaler ,may be LL_SPI_BAUDRATEPRESCALER_DIV2, LL_SPI_BAUDRATEPRESCALER_DIV4,
                                                LL_SPI_BAUDRATEPRESCALER_DIV16,LL_SPI_BAUDRATEPRESCALER_DIV32,
                                                LL_SPI_BAUDRATEPRESCALER_DIV64,LL_SPI_BAUDRATEPRESCALER_DIV128,
                                                LL_SPI_BAUDRATEPRESCALER_DIV256
@unsigned short lsbFirst- may be LL_SPI_LSB_FIRST, LL_SPI_MSB_FIRST
@unsigned short ssHighPol - the polarity of SS pin.1- active HIGH, 0- active LOw;

*/


short DLL_EXPORT setupSpi (statesHandle * states,HANDLE* pHidHandle,
                            unsigned short clkPolarity, unsigned short clkPhase,
                            unsigned short baudRate, unsigned short lsbFirst,
                            unsigned short ssHighPol) {
    unsigned char reportBufferHost[68]={0}; //incoming data for host
    unsigned char reportBufferDev[68]={0};  //outgoing data for device
    DWORD bytesWritten;
    memset(reportBufferHost,0x00,68);
    //[ "setup_spi_dev"(2b)  | phase(2b) | polarity(2b) | baudrate(2b) |lsbFirst(2b)| ss_high(2b)]
    reportBufferHost[0] = setup_spi_dev;
    memcpy(reportBufferHost+2, &clkPhase,2);
    memcpy(reportBufferHost+4, &clkPolarity,2);
    memcpy(reportBufferHost+6, &baudRate,2);
    memcpy(reportBufferHost+8, &lsbFirst,2);
    memcpy(reportBufferHost+10, &ssHighPol,2);


      if (!writePacketToUsbDev(pHidHandle, reportBufferHost,  &bytesWritten )) {

           return GetLastError();
    }
       if (!readPacketFromUsbDev(pHidHandle, reportBufferDev,   &bytesWritten )) {

              return GetLastError();
       }
       return reportBufferHost[0];


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
