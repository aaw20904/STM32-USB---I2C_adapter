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
typedef struct {
		unsigned char typeOfAction;
		unsigned char* buffPtr; //changes during exec.
		unsigned short numUsbTransations;
		unsigned short numBytesToTransaction;
		unsigned char*  reassembledDataArray; //not changed
		unsigned char slaveAddress;
} statesHandle;

char messageBuffer[1024];


/*
H A R D W A R E   N O T E:
The board - STM32F103 , well known as "blue pill".The I2C2 works as master device.
The I2C1 works as slave device.
The SPI1 always works as a master in full-duplex mode.
*/
 /**Open a virtual com port (VCP).Through this port
 the library communicates with USB to I@C, SPI adapter  */
int   openDevice (HANDLE* hSerial, LPCSTR pathToPort) {
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
int   closeDevice (HANDLE* hSerial) {
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
int   resetI2cSetBothSpeedAndSlaveAddr(statesHandle * states,HANDLE* pHidHandle,unsigned int speedBps,unsigned char slaveAddress) {
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

short   readPacketFromUsbI2cAdapter(statesHandle * states,
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
short   sendPacketToUsbI2cAdapter(statesHandle * states,
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

short   readLastSlaveI2cReceivedPacket (statesHandle * states,
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
     states->numUsbTransations = temp16 >> 6;  /// divided by 64

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


short   writeSlaveI2cTransmitterBuffer(statesHandle * states,
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


///the function to send data through the SPI1 interface.
//N O T E:  The SPI1 works ALWAYS in full duplex mode in this library
/*
  @statesHandle * states - a pointer to a states structure,
  @HANDLE* pHidHandle - a handle of opened VCP port,
  @unsigned char* buffer - from where the data be read ,
  @unsigned short amountOfData - amount of data to be sent

*/

  short  sendPacketToUsbSpiAdapter(statesHandle * states,
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

}///the function to read data from SPI1
//N O T E:  The SPI1 works ALWAYS in full duplex mode in this library
/*
  @statesHandle * states - a pointer to a states structure,
  @HANDLE* pHidHandle - a handle of opened VCP port,
  @unsigned char* buffer -  where the data be written ,
  @unsigned short amountOfData - amount of data to be read

*/

short   readPacketFromUsbSpiAdapter(statesHandle * states,
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
///the function to write/read simultaneously
//N O T E:  The SPI1 works ALWAYS in full duplex mode in this library
/*
  @statesHandle * states - a pointer to a states structure,
  @unsigned char* txBuffer - wrom where the data be read,
  @unsigned char* rxBuffer - where the data be written,
  @unsigned short amountOfData - amount of data of transaction

*/



 short   fullDuplexSpiTransaction(statesHandle * states,
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
     //12)send the 'read_from_i2c_dev' to the USB device:
            reportBufferDev[0] =  data_to_host;
    if (!WriteFile(*pHidHandle, reportBufferDev, 64, &bytesProcessed, NULL)) {

           return GetLastError();
       }
    //13)read the last piece of data from slave:
     if (!ReadFile(*pHidHandle, reportBufferHost, 64, &bytesProcessed, NULL)) {

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


short   setupSpi (statesHandle * states,HANDLE* pHidHandle,
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


      if (!WriteFile(*pHidHandle, reportBufferHost, 64, &bytesWritten, NULL)) {

           return GetLastError();
    }
       if (!ReadFile(*pHidHandle, reportBufferDev, 64, &bytesWritten, NULL)) {

              return GetLastError();
       }
       return reportBufferHost[0];


}


