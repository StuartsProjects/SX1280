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

///These are the pin definitions for one of my own boards, be sure to change them to match
//your own setup

#define NSS 10
#define RFBUSY A1
#define NRESET 9
#define LED1 8
#define DIO1 2
#define DIO2 -1                 //not used 
#define DIO3 -1                 //not used                      
#define BUZZER A0               //connect a buzzer here if wanted

//*******  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  

#define Bandwidth LORA_BW_0400                   //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate

#define start_power 12                           //Start power for transmissions in dBm, must be highest power
#define end_power -18                            //end power for transmissions in dBm

#define ThisNode 'L'                             //identifies the trasnmitter node  
#define Batch 0x3939                             //for future applications, not used here   

#define mode_delaymS 2000                         //mS delay between test sequences


#define packet_delay 1000                         //mS delay between packets

#define TXBUFFER_SIZE 16                          //default TX buffer size  
#define RXBUFFER_SIZE 16                         //default RX buffer size  

 



