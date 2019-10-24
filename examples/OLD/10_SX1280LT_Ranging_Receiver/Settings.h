/*
******************************************************************************************************

LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson

http://www.LoRaTracker.uk

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
#define RFBUSY 3
#define NRESET 9
#define DIO1 2
#define DIO2 -1
#define DIO3 -1
#define BUZZER A0


#define SX1280_TXBUFF_SIZE 32
#define SX1280_RXBUFF_SIZE 32


//*******  Setup Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t  Frequency = 2445000000;          //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes  

#define Bandwidth LORA_BW_0400                   //LoRa bandwidth
#define SpreadingFactor LORA_SF10                //LoRa spreading factor
#define CodeRate LORA_CR_4_8                     //LoRa coding rate
#define RangingTXPower 0                         //ranging reply power

#define TXaddress 16                             //Ranging address the receiver accepts, must match setting in requestor 
#define CalibrationSF10BW400 10120               //calibration value for ranging, SF10, BW400, see table "Introduction to Ranging"      
#define waittimemS 5000                          //wait this long for packet before assuming timeout
#define rangingRXTimeoutmS 10000                 //SX1280 ranging RX timeout in mS


