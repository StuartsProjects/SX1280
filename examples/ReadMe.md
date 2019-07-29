The Examples folder contains a series of test programs for the SX1280. The programs are described at the bottom of the page. Its recommended to use the **SX1280_LoRa_Register_Test** program first to test the device connections are correct. 

A common feature of the programs is the uses of a series of defines in the settings.h file. These defines determine the frequency used, the bandwidth the spreading factor and coding rate used, for example;


    #define Frequency 2445000000        //frequency of transmissions    
    #define Bandwidth LORA_BW_0400      //LoRa bandwidth    
    #define SpreadingFactor LORA_SF10   //LoRa spreading factor    
    #define CodeRate LORA_CR_4_5        //LoRa coding rate


The full list of options you can set are;

    //LoRa bandwidths
    #define LORA_BW_0200  //203khz
    #define LORA_BW_0400  //406khz
    #define LORA_BW_0800  //812khz
    #define LORA_BW_1600  //1625khz

.

    ////LoRa spreading factors
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

##Buffers

The data to be sent is loaded into an array called TXBUFFER, the length can be varied to suit, with a maximum of 256 bytes for LoRa and 128  bytes for FLRC. PAckets received by the device are pulled from the FIFO and moved into the array called RXBUFFER.


##Packet Addressing

LoRa is a two way technology, each device is a transceiver. Most often on a particular frequency there will be one transmitter and one receiver. However, this may not always be the case and there could be several nodes in use on the same frequency. 

In order to keep the software simple and allow for the receipt of signals from multiple receivers or directed commands to a particular node, a basic addressing scheme has been implemented and is used by some programs. The are library routines to send and receive packets in addressed and non-addressed format so you choose which to send. When using addressed mode regardless of the data content of the actual payload (in TXBUFFER) each payload is preceded in the transmitted packet with the 3 control bytes. In general the control bytes have been restricted to ASCII printable characters so that they can be shown directly on a terminal monitor, the 3 bytes are;

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


**SX1280\_LoRa\_Register_Test** - Checks to see if a SX1280 device can be found. prints a list of register read. 

**SX1280LT\_LoRa\_Simple_TX** - Transmits a series of LoRa packets according to the LoRa parameters in the settings.h file. Used together with matching RX program

**SX1280LT\_LoRa\_Simple\_RX** - Receives a LoRa packets according to the LoRa parameters in the settings.h file. Results displayed in IDE serial monitor. Used together with matching TX program.

**SX1280LT\_LoRa\_Link\_Test\_TX** - This used for testing the sensitivity of links or antennas etc. The transmitter sends a sequence of packets starting at a specified power (in settings.h) and decrease the power used to send the packet by 1dBm at a time. The packet contains the ASCII representation of the packet power such a +10 for 10dBm, +01 for 1dBm and -10 for -10dBm. The receiver prints these ASCII values so you can see at what power level the link fails. Use the SX1280LT_LoRa_Simple_RX program to receive the packets. This program used addressed packets.
