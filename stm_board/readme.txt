This software was written in STM32CubeIDE.The STM32F103C8 was used , it also known as "blue pill".
*****************************
The I2C master is I2C2, used pins are:
PB10->SCL
PB11->SDA
***************************
The I2C slave is I2C1,used pins are:
PB6->SCL
PB7->SDA
**************************
The master SPI1, used pins are:
PA4->SS
PA5->SCK
PA6->MISO
PA7->MOSI
******************************
I use LL libraries for initialization and set mode of I2C and SPI interfaces.The interrupt handlers
and business logic (main code) was written in pure C without libraries, because I was afraid that HAL will consume RAM/FLASH memory. The HAL library was used only for USB CDC devices (it is too complex to develop our own USB stack). 

NOTE: The full Cube project is inside the ZIP archive
