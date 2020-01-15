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
#define LED1 8
#define NSS 10
#define RFBUSY A3
#define NRESET 6
#define DIO1 5                                  //used as interrupt out from SX1280 
#define DIO2 -1                                 //not used
#define DIO3 -1                                 //not used
#define BUZZER -1                               //not used                             
#define RXEN A5                                 //for modules with RX TX switching 
#define TXEN A4                                 //for modules with RX TX switching 


//*******  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_0400                   //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate

#define PowerTX  1                               //power for transmissions in dBm

#define packet_delay 1000                        //mS delay between packets

#define TXBUFFER_SIZE 16
#define RXBUFFER_SIZE 16
