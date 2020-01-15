/*
******************************************************************************************************

lora Programs for Arduino

Copyright of the author Stuart Robinson

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

Changes:

To Do:

******************************************************************************************************
*/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, be sure to change them to match
//your own setup

#define NSS 10
#define RFBUSY A1
#define NRESET 9
#define LED1 8
#define DIO1 2
#define DIO2 -1                 //not used 
#define DIO3 -1                 //not used   
#define BUZZER 3                                        


//*******  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes

#define Bandwidth LORA_BW_0200                   //LoRa bandwidth for longest time packet
#define SpreadingFactor LORA_SF12                //LoRa spreading factor for longest time packet
#define CodeRate LORA_CR_4_8                     //LoRa coding rate for longest time packet
 
#define PowerTX 10                               //transmit power to use 

#define packet_delay 1000                        //mS delay between packets

#define TXBUFFER_SIZE 128                         //default TX buffer size  
#define RXBUFFER_SIZE 16                          //default RX buffer size  

 



