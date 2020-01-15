/*
******************************************************************************************************

  lora Programs for Arduino

  Copyright of the author Stuart Robinson 24/10/19

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  The program listens for incoming packets using the LoRa settings in the 'Settings.h' file. The pins to
  access the SX1280 need to be defined in the 'Settings.h' file also.

  The program transmits a packet without using a buffer, the LoRa device FIFO is filled direct with
  variables. Use the companion program SX1280LT_LoRa_Bufferless_RX to receive them.

  The numbers sent are;

  51.23456       (float)
  -3.12345       (float)
  2445000000     (uint32_t)
  -100000        (int32_t)
  30000          (uint16_t)
  -30000         (int16_t)
  200            (uint8_t)
  -100           (int8_t)


  Changes:

  To Do:

******************************************************************************************************/

#define programversion "V1.0"
#define Serial_Monitor_Baud 115200

#include <SPI.h>
#include "Settings.h"
#include <SX1280LT.h>

SX1280Class SX1280LT;

boolean SendOK;
int8_t TestPower;
uint8_t TXPacketL;

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
  Serial.print(F("  SentOK"));
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
  digitalWrite(LED1, LOW);                                 //this leaves the LED on slightly longer for a packet error
}


bool Send_Test_Packet()
{
  float latitude, longitude;

  latitude = 51.23456;
  longitude = -3.12345;

  SX1280LT.startWriteFIFO();
  SX1280LT.writeFloat(latitude);
  SX1280LT.writeFloat(longitude);
  SX1280LT.writeUint32(Frequency);
  SX1280LT.writeInt32(-100000);
  SX1280LT.writeUint16(30000);
  SX1280LT.writeInt16(-30000);
  SX1280LT.writeUint8(200);
  SX1280LT.writeInt8(-100);
  SX1280LT.endWriteFIFO();
  digitalWrite(LED1, HIGH);
  SendOK = SX1280LT.sendFIFOLoRa(1000, PowerTX, DIO1);
  digitalWrite(LED1, LOW);
  return SendOK;
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
  SX1280LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 128, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
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

  Serial.println(F("15_SX1280LT_LoRa_Bufferless_TX Starting"));

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

  setup_LoRa();

  Serial.println(F("Transmitter ready"));
  Serial.println();
}

