//Settings.h


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
#define DIO1 2                                  //used as interrupt out from SX1280 
#define DIO2 -1                                 //not used
#define DIO3 -1                                 //not used
#define BUZZER -1                               //not used                         


//*******  Setup FLRC Test Parameters Here ! ***************

//FLRC Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  

#define BandwidthBitRate FLRC_BR_0_260_BW_0_3    //FLRC bandwidth and bit rate, 1.3Mbs               
#define CodingRate FLRC_CR_1_2                   //FLRC coding rate
#define BT RADIO_MOD_SHAPING_BT_1_0              //FLRC BT
#define Sample_Syncword 0x01234567               //FLRC uses syncword

#define start_power 1                            //Start power for transmissions in dBm
#define waittimemS 5000                          //wait this long in mS for packet before assuming timeout

#define packet_delay 500                         //mS delay between packets

#define TXBUFFER_SIZE 16                         //default TX buffer size  
#define RXBUFFER_SIZE 16                         //default RX buffer size  

 



