## Examples

The Examples folder contains a series of test programs for the SX1280. The programs are described at the bottom of the page. Its recommended to use the **SX1280_LoRa_Register_Test** program first to test the device connections are correct. 

A common feature of the programs is the uses of a series of defines in the settings.h file. These defines determine the frequency used, the bandwidth the spreading factor and coding rate used, for example;


    #define Frequency 2445000000        //frequency of transmissions    
    #define Bandwidth LORA_BW_0400      //LoRa bandwidth    
    #define SpreadingFactor LORA_SF10   //LoRa spreading factor    
    #define CodeRate LORA_CR_4_5        //LoRa coding rate

<br>

### LoRa Modem Settings

    //LoRa bandwidths
    #define LORA_BW_0200  //203khz
    #define LORA_BW_0400  //406khz
    #define LORA_BW_0800  //812khz
    #define LORA_BW_1600  //1625khz

.

    //LoRa spreading factors
    #define LORA_SF5
    #define LORA_SF6 
    #define LORA_SF7
    #define LORA_SF8
    #define LORA_SF9
    #define LORA_SF10 
    #define LORA_SF11
    #define LORA_SF12
.

    //LoRa coding rates
    #define LORA_CR_4_5
    #define LORA_CR_4_6
    #define LORA_CR_4_7
    #define LORA_CR_4_8


### FLRC Modem Settings

    //FLRC bandwidth and bit rate
    #define FLRC_BR_1_300_BW_1_2 0x45   //1.3Mbs  
    #define FLRC_BR_1_000_BW_1_2 0x69   //1.04Mbs 
    #define FLRC_BR_0_650_BW_0_6 0x86   //0.65Mbs
    #define FLRC_BR_0_520_BW_0_6 0xAA   //0.52Mbs
    #define FLRC_BR_0_325_BW_0_3 0xC7   //0.325Mbs
    #define FLRC_BR_0_260_BW_0_3 0xEB   //0.26Mbs`
.

    //FLRC coding rate`
    #define FLRC_CR_1_2  0x00           //coding rate 1:2
    #define FLRC_CR_3_4  0x02           //coding rate 3:4
    #define FLRC_CR_1_0  0x04           //coding rate 1 

<br>
<br>

### GFSK Modem Settings

Not yet tested

### BLE Modem Settings

Not yet tested

<br>
<br>
## Buffers

The data to be sent is loaded into an array called TXBUFFER, the length can be varied to suit, with a maximum of 256 bytes for LoRa and 128  bytes for FLRC. Packets received by the device are pulled from the FIFO and moved into the array called RXBUFFER.


## Packet Addressing

LoRa is a two way technology, each device is a transceiver. Most often on a particular frequency there will be one transmitter and one receiver. However, this may not always be the case and there could be several nodes in use on the same frequency. 

In order to keep the software simple and allow for the receipt of signals from multiple receivers or directed commands to a particular node, a basic addressing scheme has been implemented and is used by some programs. There are library routines to send and receive packets in addressed and non-addressed format so you choose which to send. When using addressed mode regardless of the data content of the actual payload (in TXBUFFER) each packet sent has 3 control bytes at the beginning of the packet. In general the control bytes have been restricted to ASCII printable characters so that they can be shown directly on a terminal monitor. The 3 bytes are;

**Packet type**. This either describes the content of the packet, which could be a GPS location payload or is a command to do something and there is no payload. Details of the packet types defined are in the library file 'SX1280LT_Program_Definitions.h'

**Packet Destination**. The node number that the packet is destined for.

**Packet Source**. The node number that the packet was sent from.

The destination and source packet bytes mean that node ‘2’ (could be your base station receiver) can send a command that only station ‘3’ will respond to. This command could be a reset command, a request to turn on and off certain transmission types etc. Node ‘3’ can be set-up so that it only accepts commands from a particular node.

In addressed mode the 3 control bytes are automatically stripped from each received packet and the rest of the data is placed in an array called RXBUFFER. 

An example of the 3 control bytes from a tracker would be;

T*2

Which means there is a test packet (T) its been sent as a broadcast (*) and its from node 2.

This simple protocol can be used to queue requests to intermittent receivers. For instance the high altitude balloon tracker software assumes that the in flight tracker does not listen for commands continuously; it only listens for incoming packets for short periods in order to minimise power consumption. At the beginning of a listen period (which may only be a few seconds) the tracker sends a ready (clear to send) packet. The receiver is normally not so power constrained so is left in permanent listen mode. The receiver has been given a queued request (could be to release a payload command for instance) and is waiting for the ready packet from the tracker. When the ready packet is received, the receiver sends the queued command and listens for an acknowledge packet. If the receiver does not receive an acknowledge it will keep queuing the request.

The high altitude balloon tracking software uses the packet addressing to implement a command and control interface between the ground receiver and the remote balloon.  

<br>

For the example programs set the serial monitor baud rate to 115200 baud.

**1 SX1280 LoRa RegisterTest** - Checks to see if a SX1280 device can be found. Prints a list of registers read. Use this program first to check the SX1280 module is wired up correctly.  

**2 SX1280LT LoRa Simple TX** - Transmits a series of LoRa packets according to the LoRa parameters in the settings.h file. The LED will flash when a packet is transmitted. Used together with matching RX program. 

**3 SX1280LT LoRa Simple RX** - Receives LoRa packets according to the LoRa parameters in the settings.h file. Results, RSSI, SNR, errors etc are  displayed in the Arduino IDE serial monitor. The LED will flash when a packet is received, you can add and enable a buzzer too. Used together with matching TX program.  

**4 SX1280LT LoRa Link Test TX** - This used for testing the sensitivity of links or antennas etc. The transmitter sends a sequence of packets starting at a specified power (in settings.h) and decreasing the power used to send the packet by 1dBm at a time. The packet contains the ASCII representation of the packet power such a +10 for 10dBm, +01 for 1dBm and -10 for -10dBm. The receiver prints these ASCII values so you can see at what power level the link fails. Use the '3 SX1280LT_LoRa_Simple_RX' program to receive the packets. This program used addressed packets. The principles of this type of link testing are discussed, for 434Mhz LoRa in the document '[Testing and Comparing - December 2018](https://github.com/LoRaTracker/Link-Tester2/blob/master/Testing%20and%20Comparing%20-%20December%202018.pdf)' see the section on 'Test Software – Descending Power Tests' in particular. 

**5 SX1280LT LoRa FrequencyCounter Check_TX**

This program generates a LoRa packet that is around 5 seconds long. The program can be used for checking the frequency programmed for the SX1280 is as expected and for measuring the power output level.

**6 SX1280LT LoRa RX Frequency Error Check**

This receiver program demonstrates the use of the ability of the SX1280 to measure the frequency error, in relation to the receivers frequency, of an incoming packet. Prints the frequency error over an average of 10 packets to the serial monitor. Can be used to ensure that modules are close together in frequency. 

**7 SX1280LT LoRa Simple TX Implicit**

Transmitter program that demonstrates the use of implicit header mode packets that are fixed length. There is no packet header in implicit mode so the packet length and presence of packet CRC must be configured the same as for the matching RX program. 


**8 SX1280LT LoRa Simple RX Implicit**

Receiver program that demonstrates the use of implicit header mode packets that are fixed length. There is no packet header in implicit mode so the packet length and presence of packet CRC must be configured the same as for the matching TX program.

**9 SX1280LT LoRa TX and Sleep**

The SX1280 has features that allow the register state of the device to be saved internally before putting the device into low current deep sleep. When the device is woken up, by pulsing the NSS select pin, the device reloads all the internal registers and the device can continue as if it had never been asleep. The feature avoids having to reconfigure the device on wakeup. The program sends a packet 'Hello World1' goes into deep sleep for around 10 seconds, wakes up an attempts to transmit a 'Hello World2'. Packet. If the wakeup is working correctly then the '3 SX1280LT LoRa Simple RX' program should show both packets arriving.  


### Ranging Programs

These are to be considered a work in progress, not intended for any mission critical use. During some recent balloon flights the ranging function would occasionally return invalid results, not yet sure why. 


**10 SX1280LT Ranging Master**

This program transmits the ranging requests. It will attempt 5 ranging requests and average the results of the number of valid responses. The ranging requester can send requests to specific receivers, see the SX1280 datasheet for details. In these examples an address of 16 is used for both RX and TX so that there is a direct match.

The Ranging Master program can display the results on an SSD1306 OLED display, handy for portable use. To enable this option in the program comment in the //#define ENABLEDISPLAY line in the Settings.h file by removing the two // characters at the start of the line. 

**11 SX1280LT Ranging Slave**

This is the program for the slave node that will respond to ranging requests. It waits for incoming ranging requests of the correct address and if valid responds with a packet back to the master. 


**12 SX1280LT Ranging Calibration Checker**

This is used to calibrate the ranging function, see the separate document 'Ranging Calibration.md' for details on how to use it. Needs the ranging receiver running.


**13 SX1280LT FLRC Simple RX**

Receives Fast Long Range Communications (FLRC) mode packets according to the LoRa parameters in the settings.h file. Results, RSSI, SNR, errors etc are  displayed in the Arduino IDE serial monitor. The LED will flash when a packet is received, you can add and enable a buzzer too. Used together with matching TX program. According to the SX1280 data sheet for LoRa at SF5 and bandwidth 1625khz, the effective data rate is 203kbps with a sensitivity of -99dBm. For FLRC mode at a bit rate of 1.3Mbps and bandwidth of 1.2Mhz the effective data rate is 975kbps at the same sensitivity (-99dBm) as 203kbps LoRa.    

**14 SX1280LT FLRC Simple RX**

Transmits Fast Long Range Communications (FLRC) mode packets according to the LoRa parameters in the settings.h file. The LED will flash when a packet is transmitted, Used together with matching RX program. 

### Bufferless Programs


**15 SX1280LT LoRa Bufferless TX**

This is a demonstration of writing outgoing packets direct to the SX1280 FIFO, with no memory buffer required. Variables such as integers and floats can be written. The order the variables are written to the FIFO on the transmitter must match the order they are read from the FIFO in the matching RX program.

**16 SX1280LT LoRa Bufferless RX**

This is a demonstration of reading incoming packets direct from the SX1280 FIFO, with no memory buffer required. Variables such as integers and floats can be read. The order the variables are read from the FIFO on the receiver must match the order they are written to the FIFO in the matching TX program.

### RX and TX Enable Switching


**17 SX1280LT LoRa Simple RX RXTXEnable**

Some SX1280 devices such as the Ebyte E28-2G4M20S have RX enable and TX enable pins that must be manually switched for transmit and receive operations. This program and the matching RX program demonstrates the use of the RX TX enable switching. In essense all that need to be down is to define the RXEN and TXEN pins in the Settings.h file and call this function at the initial configuration; **SX1280LT.rxtxInit(RXEN, TXEN)**. For RX mode RXEN is switched high and TXEN is low. For TX mode TXEN is switched high and RXEN is low.   

**17 SX1280LT LoRa Simple TX RXTXEnable**

Matching transmitter program for the '17 SX1280LT LoRa Simple RX RXTXEnable' program. 


Enjoy. 
<br>
<br>
### Stuart Robinson
### October 2019


