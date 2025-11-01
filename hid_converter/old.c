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
#define reset_interface 26
#define setup_interface_i2c 27
#define read_last_stub_rx_i2c 28
#define write_tx_stub_buffer_i2c  29
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
typedef struct {
		unsigned char typeOfAction;
		unsigned char* buffPtr; //changes during exec.
		unsigned short numUsbTransations;
		unsigned short numBytesToTransaction;
		unsigned char*  reassembledDataArray; //not changed
		unsigned char slaveAddress;
} statesHandle;

char messageBuffer[1024];




int resetI2cSetBothSpeedAndSlaveAddr(statesHandle * states,HANDLE* pHidHandle,unsigned int speedBps,unsigned char slaveAddress) {
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

///function for sending data through I2C
short readPacketFromUsbI2cAdapter(statesHandle * states,
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


short writeSlaveI2cTransmitterBuffer(statesHandle * states,
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

short readLastSlaveI2cReceivedPacket (statesHandle * states,
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

///function for sending data through I2C
short sendPacketToUsbI2cAdapter(statesHandle * states,
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


int openDevice (HANDLE* hSerial, LPCSTR pathToPort) {
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
        // 1) Open the COM port
    *hSerial = CreateFileA(
        pathToPort,  /*  "\\\\.\\COM3"  */
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

    //set pararmeters
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
