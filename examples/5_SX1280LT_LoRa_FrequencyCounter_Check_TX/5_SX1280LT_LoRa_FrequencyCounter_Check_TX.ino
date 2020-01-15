/*
******************************************************************************************************

  lora Programs for Arduino

  Copyright of the author Stuart Robinson 24/10/19

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

  This repeatadly transmits a long packet, approx 6 seconds,  so that there is enough air time to
  measure the output on a frequency counter.


******************************************************************************************************
*/

#define programversion "V1.0"
#define Serial_Monitor_Baud 115200

#include <SPI.h>
#include "Settings.h"
#include <SX1280LT.h>

SX1280Class SX1280LT;

boolean SendOK;
int8_t TestPower;
uint8_t TXPacketL;
float temp;

void loop()
{
  digitalWrite(LED1, HIGH);
  Serial.print(PowerTX);
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
  float frequency;
  uint8_t bufffersize;
  uint8_t buff[] = "Hello World!1234Hello World!1234Hello World!1234Hello World!1234Hello World!1234Hello World!1234Hello World!1234Hello World!1234";

  if (sizeof(buff) > TXBUFFER_SIZE)                   //check that defined buffer is not larger than TX_BUFFER
  {
    bufffersize = TXBUFFER_SIZE;
  }
  else
  {
    bufffersize = sizeof(buff);
  }

  TXPacketL = bufffersize;

  SX1280LT.printASCIIPacket(buff, bufffersize);

  digitalWrite(LED1, HIGH);

  frequency = SX1280LT.getFreqInt();
  Serial.println();
  Serial.print(F("Frequency "));
  Serial.print(frequency);
  Serial.print(F("hz"));
  Serial.flush();

  if (SX1280LT.sendPacketLoRa(buff, bufffersize, 10000, PowerTX, DIO1))    //timeout is 10 seconds
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


void setup_LoRa()
{
  SX1280LT.setStandby(MODE_STDBY_RC);
  SX1280LT.setRegulatorMode(USE_LDO);
  SX1280LT.setPacketType(PACKET_TYPE_LORA);
  SX1280LT.setRfFrequency(Frequency, Offset);
  SX1280LT.setBufferBaseAddress(0, 0);
  SX1280LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  SX1280LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
}


void setup()
{
  uint32_t frequency;
  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);

  Serial.begin(Serial_Monitor_Baud);
  Serial.println();
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println();

  Serial.println(F("5_SX1280LT_LoRa_Simple_TX Starting"));

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

  frequency = SX1280LT.getFreqInt();
  Serial.println();
  Serial.print(F("Frequency "));
  Serial.print(frequency);
  Serial.print(F("hz"));
  Serial.flush();

  setup_LoRa();

  frequency = SX1280LT.getFreqInt();
  Serial.println();
  Serial.print(F("Frequency "));
  Serial.print(frequency);
  Serial.print(F("hz"));
  Serial.flush();

  Serial.print(F("Transmitter ready - TXBUFFER_SIZE "));
  Serial.println(TXBUFFER_SIZE);
  Serial.println();
}

