# SX1280 - Semtech 2.4ghz LoRa Transceivers

This is a a repository for my Arduino library for the Semtech SX1280 LoRa device. This device is avaialable in modules from NiceRF and EByte. The code will support the Ebyte 14 pin modules but not the 16 pin modules that require RX and TX switching.  

The SX1280 operates in the 2.4Ghz band. In addition to having a LoRa modem the SX1280 can send GFSK and FLRC (Fast Long Range Communication) packets. 
These SX1280 devices have been available since 2017 and can be used for both point to point applications and distance measurements using the built in ranging function. 

The \Examples folder contains basic transmitter and receiver programs for both LoRa and FLRC. There are link test transmitter programs as well, these program allows the performance of a link to be measured. See the \PointToPoint folder for more details on how to use these programs. 

The ranging function measures the time of flight of of a packet exchange between the inmitiator and receiver. See the \Ranging folder for more details. The \Ranging folder contains a report of the long distance capability of the SX1280 device, where distances of 40km were recorded for both point to point and ranging functions.  


### Stuart Robinson
### GW7HPW
### July 2019