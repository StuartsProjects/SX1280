/*
******************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 09/07/19

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

/*
******************************************************************************************************

Program Operation

This program checks that an SX1280 LoRa device can be accessed. There should be two short LED flashes   
at startup. If the SX1280 is detected there will be two more LED flashes and the contents of the 
registers from 0x900 to 0x9F are printed, this is a copy of a typical printout below. 
  
If the SX1280 is not detected the LED will flashes rapidly. 

SX1280_LoRa_Register_Test Starting
Device found
Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
0x900  80 0C 7B 02 20 FA C0 00 00 80 00 00 00 00 00 FF 
0x910  FF FF 00 00 00 19 00 00 00 19 87 65 43 21 7F FF 
0x920  FF FF FF 0C 70 37 0A 50 D0 80 00 C0 5F D2 8F 0A 
0x930  00 C0 00 00 00 24 00 21 28 B0 30 09 1A 59 70 08 
0x940  58 0B 32 0A 14 24 6A 96 00 18 00 00 00 00 00 00 
0x950  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
0x960  00 00 00 00 00 00 00 00 00 00 FF FF FF FF FF FF 
0x970  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 04 
0x980  00 0B 18 70 00 00 00 4C 00 F0 64 00 00 00 00 00 
0x990  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
0x9A0  00 08 EC B8 9D 8A E6 66 06 00 00 00 00 00 00 00 
0x9B0  00 08 EC B8 9D 8A E6 66 06 00 00 00 00 00 00 00 
0x9C0  00 16 00 3F E8 01 FF FF FF FF 5E 4D 25 10 55 55 
0x9D0  55 55 55 55 55 55 55 55 55 55 55 55 55 00 00 00 
0x9E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
0x9F0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
 

******************************************************************************************************
*/

//These are the pin definitions for one of my own boards, be sure to change them to match
//your own setup

#define LED1 8                                  //for on board LED
#define RFBUSY 3                                //SX1280 RFBUSY pin  
#define NSS 10                                  //SX1280 device select
#define NRESET 9                                //SX1280 reset pin
#define DIO1 -1                                 //not used 
#define DIO2 -1                                 //not used
#define DIO3 -1                                 //not used

#define programversion "V1.0"
#define Serial_Monitor_Baud 115200              //serial monitor baud rate 

#include <SPI.h>
#include <SX1280LT.h>

SX1280Class SX1280LT;

void loop()
{
 uint32_t frequency;
 frequency = SX1280LT.getFreqInt();
 Serial.print(F("Frequency at reset "));
 Serial.println(frequency);
 
 digitalWrite(LED1, HIGH);
 SX1280LT.printRegisters(0x900, 0x9FF);
 SX1280LT.setRfFrequency(2445000000, 0);
 frequency = SX1280LT.getFreqInt();
 Serial.print(F("Changed Frequency "));
 Serial.println(frequency);
 Serial.println();
 Serial.println();
 digitalWrite(LED1, LOW);
 delay(5000);
 SX1280LT.resetDevice();
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  unsigned int index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void setup()
{
  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);

  Serial.begin(Serial_Monitor_Baud);
  Serial.println();
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println();

  Serial.println(F("1_SX1280LT_LoRa_Register_Test Starting"));

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  if (SX1280LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3))
  {
    Serial.println(F("Device found"));
    led_Flash(2, 125);
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                            //long fast speed flash indicates device error
    }
  }

  
}

