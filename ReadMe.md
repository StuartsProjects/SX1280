# SX1280 - Semtech 2.4ghz LoRa Transceivers

This is a a repository for my Arduino library for the Semtech SX1280 LoRa device. This device is avaialable in modules from NiceRF and EByte. The code will support the Ebyte 14 pin modules but not the 16 pin modules that require RX and TX switching.  

The SX1280 operates in the 2.4Ghz band. In addition to having a LoRa modem the SX1280 can send GFSK and FLRC (Fast Long Range Communication) packets. 
These SX1280 devices have been available since 2017 and can be used for both point to point applications and distance measurements using the built in ranging function. 

The \Examples folder of the library contains basic transmitter and receiver programs for both LoRa and FLRC. There are link test transmitter programs as well, these program allows the performance of a link to be measured.  

The ranging function measures the time of flight of of a packet exchange between the initiator and receiver. See the \Ranging folder for more details. There is a GITHUB repository where there are reports of the  distance testing of the SX1280 device, its located here;

[https://github.com/LoRaTracker/SX1280_Testing](https://github.com/LoRaTracker/SX1280_Testing "https://github.com/LoRaTracker/SX1280_Testing ")

Distances of up to 85km were recorded for ranging and 89km for point to point. 

###Installation

To install the library select the 'Clone or download' button on the main Github page, then select 'Download Zip'. In the Arduino IDE select 'Sketch' then 'Include Library'. Next select 'Add .ZIP library' and browse to and select the ZIP file you downloaded, its called 'SX1280-master.zip'.




### Stuart Robinson
### GW7HPW
### July 2019