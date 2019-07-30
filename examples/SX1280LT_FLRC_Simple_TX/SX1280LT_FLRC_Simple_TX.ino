#define programversion "V1.0"
#define Serial_Monitor_Baud 115200


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

/*
******************************************************************************************************

  Program Operation

  The FLRC settings to do are specified in the 'Settings.h' file.


******************************************************************************************************
*/



#include <SPI.h>
#include "Settings.h"
#include <SX1280LT.h>

SX1280Class SX1280LT;

boolean SendOK;
int8_t TestPower;
uint8_t TXPacketL;

void loop()
{
  TestPower = start_power;
  digitalWrite(LED1, HIGH);
  Serial.print(TestPower);
  Serial.print(F("dBm "));
  Serial.print(F("TestPacket> "));
  Serial.flush();

  if (Send_Test_Packet())
  {
    packet_is_OK();
  }
  else
  {
    packet_is_Error();
  }
  Serial.println();
  delay(packet_delay);
}


void packet_is_OK()
{
  Serial.print(F(" "));
  Serial.print(TXPacketL);
  Serial.print(F(" Bytes SentOK"));
}


void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = SX1280LT.readIrqStatus();                    //get the IRQ status
  Serial.print(F("SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  SX1280LT.printIrqStatus();
  digitalWrite(LED1, LOW);                       //this leaves the LED on slightly longer for a packet error
}


bool Send_Test_Packet()
{
  uint8_t buff[] = "Hello World!";
  TXPacketL = sizeof(buff);

  SX1280LT.printASCIIPacket(buff, sizeof(buff));

  digitalWrite(LED1, HIGH);

  if (SX1280LT.sendPacketFLRC(buff, sizeof(buff), 1000, TestPower, DIO1))
  {
    digitalWrite(LED1, LOW);
    return true;
  }
  else
  {
    return false;
  }
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


void setup_FLRC()
{
  SX1280LT.setStandby(STDBY_RC);
  SX1280LT.setRegulatorMode(USE_LDO);
  SX1280LT.setPacketType(PACKET_TYPE_FLRC);
  SX1280LT.setRfFrequency(Frequency, 0);
  SX1280LT.setBufferBaseAddress(0, 0);
  SX1280LT.setModulationParams(BandwidthBitRate, CodingRate, BT);
  SX1280LT.setPacketParams(PREAMBLE_LENGTH_32_BITS, FLRC_SYNC_WORD_LEN_P32S, RADIO_RX_MATCH_SYNCWORD_1, RADIO_PACKET_VARIABLE_LENGTH, 127, RADIO_CRC_3_BYTES, WHITENING);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);              //set for IRQ on TX done and timeout on DIO1
  SX1280LT.setSyncWord1(Sample_Syncword);
}


void setup()
{
  pinMode(LED1, OUTPUT);
  led_Flash(2, 250);

  Serial.begin(Serial_Monitor_Baud);
  Serial.println();
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println();

  Serial.println(F("SX1280LT_FLRC_Simple_TX Starting"));

  if (SX1280LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3))
  {
    Serial.println(F("Device found"));
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                            //long fast speed flash indicates device error
    }
  }

  setup_FLRC();

  led_Flash(2, 250);

  Serial.println(F("FLRC Transmitter ready"));
  Serial.println();
}

