# SX1280 - Semtech 2.4GHz LoRa Transceivers

##Updated 24/10/19

This is a a repository for my Arduino library for the Semtech SX1280 LoRa device. This Semtech device is available in modules from NiceRF and EByte. 

**The Semtech SX1280 is a 3.3V logic level device, do not use with 5V logic level Arduinos.** The programs have only been tested on 3.3V 8Mhz ATMega328P and ATMega1284P processors. 

To support the modules that require a RXEnable and TXEnable pin to be set correctly when transmitting or receiving you must execute the 'rxtxInit(RXEN, TXEN)' library function to both enable the pins as outputs and carry out the appropriate switching, see the example programs. The library code has been tested with the Ebyte E28-2G4M20S module where the RXEN pin needs to be high during receive with TXEN low, and during transmit TXEN pin is high with RXEN pin low. Note that the ranging functions will not work with modules that require the external RXEnable and TXEnable switching. 

For all example programs you will need to define the following pins as a minimum for the LoRa module in the Settings.h file; NSS, RFBUSY, NRESET, DIO1. RXEN and TXEN are needed for modules that require RX and TX switching. 

Most programs use a LED as an indicator and this is defined as pin LED1. Some programs will also turn on a buzzer as an indicator this is connected to pin BUZZER. DIO2 and DIO3 are currently not used by the library. Unused pins in the Settings.h file should be defined as -1. 
The SX1280 operates in the 2.4GHz band. In addition to having a LoRa modem the SX1280 can send GFSK and FLRC (Fast Long Range Communication) packets. The library only supports LoRa and FLRC modes. These SX1280 devices have been available since 2017 and can be used for both point to point applications and distance measurements using the built in ranging function.

FLRC (Fast Long Range Communication) mode is specified as having a similar sensitivity as the fastest LoRa mode, 203kbps, but with an improved data rate of 975kbps.  

The library is now in its second revision, there are still some issues to attend to and changes to be made, see the sections 'Changes in Revision 2 Library' and 'Changes Required to Library' at the bottom of this document. 

Please do not expect basic level support on how to connect the SX1280 devices to a particular Arduino or other device, I just don't have the time to do this. The examples do work, so if for you they do not, assume there is a problem with your Arduino set-up or how you have wired the modules or that they are faulty.

If the example programs are working, but you consider the reception distance is poor at maximum power (12dBm) there is no magic voodoo configuration that will make reception 'much better'. Reception distance can vary tremendously depending on environment, what range you get with good line of sight can be 1000 (or more) times the reception distances in urban areas.

In balloon tracker flight tests distances of up to **89km** have been recorded. 


### Distance measurements

As well as providing point to point LoRa and FLRC communications the SX1280 has a ranging function which measures the time of flight of of a packet exchange between the initiator and receiver and this can be converted to a distance. 

Distances of up to **89km** were recorded for ranging, the remote device was a GPS tracker and the distance was accurate to within 100m.   

### Testing reports

There is a GITHUB repository where there are reports of the distance testing of the SX1280 device, its located here; 

[https://github.com/LoRaTracker/SX1280_Testing](https://github.com/LoRaTracker/SX1280_Testing)

### Program examples

The \examples folder of the library contains basic transmitter and receiver programs for both LoRa and FLRC. There are link test transmitter programs as well, these programs allows the performance of a link to be measured.

### SX1280 connections  

The SX1280 can operate with a UART or SPI based interface. All the example programs use the SPI interface. The SX1280 will need pin connections for NSS (select) NRESET (reset) RFBUSY (busy) and one of the interrupt out pins, DIO1 is used in the examples. The SPI connections for the SPI interface, SCK, MOSI and MISO are needed also. Most of the testing and evaluation of the SX1280 was carried out using Mikrobus compatible boards, see the boards folder in the testing GITHUB link given above for details. 

### Library installation

To install the library select the 'Clone or download' button on the main Github page, then select 'Download Zip'. In the Arduino IDE select 'Sketch' then 'Include Library'. Next select 'Add .ZIP library' and browse to and select the ZIP file you downloaded, it's called 'SX1280-master.zip'.

### Compatibility

Tested on 3.3V 8Mhz ATMega328P and ATMega1284P only. 


<br>

### Changes in Revision 2 Library 24/10/19

Corrected problems with frequency error output
<br>
Checked recovery of settings following a busy timeout error
<br>Added a clearIrqStatus(IRQ\_RADIO\_ALL) in setTX() and SetRX() functions
<br>Change GetFreqInt() to getFreqInt()
<br>Added support for RX and TX enable pin switching
<br>Added config option to setSleep()
<br>Modify setSleep() to provide for register data retention as per data sheet. 
<br>Checked setSleep() puts device in low current mode, 0.1uA. 
<br>Remove startmS, timemS from send packet routines 
<br>Changed sendBufferLoRa to sendFIFOLoRa
<br>Add packet implicit mode support and examples
<br>Moved spiInit out of class instance constructor, this now needs to be done in setup()
<br>Changes some timeout values from unit16\_t to uint32\_t
<br>Definition for WHITENING removed, use RADIO\_WHITENING\_OFF or RADIO\_WHITENING\_ON
<br>Add frequency offset in setFrequency to all examples
<br>Set all LoRa examples to SF7 and LORA\_BW\_0400 - apart from ranging which is SF10
<br>Removed SX1280DEBUG2 defines


### Changes Pending

Review the ranging features, there are occasions when a null result is returned, particularly at long distances of several kilometres.
Implement the ranging and other bandwidths and spreading factors.

Re-test FLRC effectiveness.  

<br>
<br>


### Stuart Robinson
### October 2019