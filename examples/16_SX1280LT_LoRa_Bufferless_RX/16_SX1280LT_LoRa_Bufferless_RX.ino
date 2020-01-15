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

  The program is a simple receiver that does not use a buffer to receive a packet. The SX1280 FIFO is
  accessed directly. The companion TX program sends this sequence of numbers as a payload, there are no gaps
  between the numbers;

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

uint32_t RXpacketCount;
uint16_t errors;
bool ENABLEBUZZER;

uint8_t RXPacketL;             //length of received packet
int8_t  PacketRSSI;            //RSSI of received packet
int8_t  PacketSNR;             //signal to noise ratio of received packet


void loop()
{
  SX1280LT.setRx(PERIOBASE_01_MS, 0);

  while (!digitalRead(DIO1));                    //wait for RxDone or timeout interrupt activating DIO1

  digitalWrite(LED1, HIGH);

  if (ENABLEBUZZER)
  {
    digitalWrite(BUZZER, HIGH);
  }

  SX1280LT.readPacketReceptionLoRa();
  RXPacketL = SX1280LT.readRXPacketL();
  PacketRSSI = SX1280LT.readPacketRSSI();
  PacketSNR = SX1280LT.readPacketSNR();

  if (SX1280LT.readIrqStatus() == (IRQ_RX_DONE + IRQ_HEADER_VALID + IRQ_PREAMBLE_DETECTED))
  {
    packet_is_OK();
  }
  else
  {
    packet_is_Error();
  }

  digitalWrite(LED1, LOW);

  if (ENABLEBUZZER)
  {
    digitalWrite(BUZZER, LOW);
  }

  Serial.println();
}


void packet_is_OK()
{
  uint16_t IRQStatus;
  uint8_t len;
  uint32_t Frequency_Temp;

  int32_t tempint32;
  int16_t tempint16;
  uint16_t tempuint16;
  int8_t tempint8;
  uint8_t tempuint8;
  uint8_t RXPacketL;

  float latitude, longitude;

  RXpacketCount++;

  //packet has been received, now read from the SX1280 FIFO in the correct order.
  SX1280LT.startReadFIFO();
  latitude = SX1280LT.readFloat();
  longitude = SX1280LT.readFloat();
  Frequency_Temp = SX1280LT.readUint32();
  tempint32 = SX1280LT.readInt32();
  tempuint16 = SX1280LT.readUint16();
  tempint16 = SX1280LT.readInt16();
  tempuint8 = SX1280LT.readUint8();
  tempint8 = SX1280LT.readInt8();
  SX1280LT.endReadFIFO();

  Serial.print(latitude, 5);
  Serial.print(",");
  Serial.print(longitude, 5);
  Serial.print(",");
  Serial.print(Frequency_Temp);
  Serial.print(",");
  Serial.print(tempint32);
  Serial.print(",");
  Serial.print(tempuint16);
  Serial.print(",");
  Serial.print(tempint16);
  Serial.print(",");
  Serial.print(tempuint8);
  Serial.print(",");
  Serial.print(tempint8);
  Serial.print(",RSSI,");
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Packets,"));
  Serial.print(RXpacketCount);

  Serial.print(F(",Length,"));
  Serial.print(RXPacketL);
  IRQStatus = SX1280LT.readIrqStatus();
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
}


void packet_is_Error()
{
  uint16_t IRQStatus;

  if (ENABLEBUZZER)
  {
    digitalWrite(BUZZER, LOW);
    delay(100);
    digitalWrite(BUZZER, HIGH);
  }

  IRQStatus = SX1280LT.readIrqStatus();                    //get the IRQ status
  errors++;
  Serial.print(F("PacketError,RSSI"));

  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);

  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  SX1280LT.printIrqStatus();
  digitalWrite(LED1, LOW);
  if (ENABLEBUZZER)
  {
    digitalWrite(BUZZER, LOW);
    delay(100);
    digitalWrite(BUZZER, HIGH);
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
  SX1280LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 128, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RX_DONE, 0, 0);       //set for IRQ on RX done on DIO1
}


void setup(void)
{
  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);

  Serial.begin(Serial_Monitor_Baud);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(programversion));
  Serial.println();

  Serial.println(F("16_SX1280LT_LoRa_Bufferless_RX Starting"));

  if (BUZZER >= 0)
  {
    ENABLEBUZZER = true;
    pinMode(BUZZER, OUTPUT);
    Serial.println(F("BUZZER Enabled"));
  }
  else
  {
    ENABLEBUZZER = false;
    Serial.println(F("BUZZER Not Enabled"));
  }

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

  Serial.println(F("Receiver ready"));
  Serial.println();
}



