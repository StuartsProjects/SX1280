# SX1280 - Semtech 2.4GHz LoRa Transceivers

This is a a repository for my Arduino library for the Semtech SX1280 LoRa device. This device is available in modules from NiceRF and EByte. The code will support the Ebyte 14 pin modules but not the 16 pin modules that require RX and TX switching. These modules are all 3.3V devices, do not use directly with 5V Arduinos.

The SX1280 operates in the 2.4GHz band. In addition to having a LoRa modem the SX1280 can send GFSK and FLRC (Fast Long Range Communication) packets. 
These SX1280 devices have been available since 2017 and can be used for both point to point applications and distance measurements using the built in ranging function.

### Distance measurements

As well as providing point to point LoRa communications the SX1280 has a ranging function which measures the time of flight of of a packet exchange between the initiator and receiver and this can be converted to a distance. 

Distances of up to **85km** were recorded for ranging and **89km** for point to point. 

### Testing reports

There is a GITHUB repository where there are reports of the distance testing of the SX1280 device, its located here; 

[https://github.com/LoRaTracker/SX1280_Testing](https://github.com/LoRaTracker/SX1280_Testing)

### Program examples

The \examples folder of the library contains basic transmitter and receiver programs for both LoRa and soon FLRC. There are link test transmitter programs as well, these program allows the performance of a link to be measured.

### SX1280 connections  

The SX1280 can operate with a UART or SPI based interface. All the example programs use the SPI interface. The SX1280 will need pin connections for NSS (select) NRESET (reset) RFBUSY (busy) and DIO1. The SPI connections for the SPI interface, SCK, MOSI and MISO also. Most of the testing and evaluation of the SX1280 was carried out using Mikrobus compatible boards, see the boards folder in the testing GITHUB link given above. 

### Library installation

To install the library select the 'Clone or download' button on the main Github page, then select 'Download Zip'. In the Arduino IDE select 'Sketch' then 'Include Library'. Next select 'Add .ZIP library' and browse to and select the ZIP file you downloaded, it's called 'SX1280-master.zip'.

### Compatibility

Tested on 3.3V 8Mhz ATMega328P and ATMega1284P. 

<br>
<br>


### Stuart Robinson
### July 2019