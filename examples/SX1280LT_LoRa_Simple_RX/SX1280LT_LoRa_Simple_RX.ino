//SX1280LT_LoRa_Simple_RX.ino
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

  The program listens for incoming packets using the LoRa settings in the 'Settings.h' file. The pins to
  access the SX1280 need to be defined in the 'Settings.h' file also.

  The program is a simple receiver. There is a printout of the valid packets received, these are assumed
  to be in ASCII printable text. The LED will flash for each packet received and the buzzer will sound,
  if fitted.

  Changes:

  To Do:

******************************************************************************************************/


#define programversion "V1.0"
#define Serial_Monitor_Baud 115200

#include <SPI.h>
#include "Settings.h"
#include "SX1280LT.h"

SX1280Class SX1280LT;

uint32_t RXpacketCount;
uint32_t errors;
bool ENABLEBUZZER;

uint8_t RXBUFFER[RXBUFFER_SIZE];

uint8_t RXPacketL;                               //stores length of packet received
int8_t  PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio of received packet


void loop()
{
  SX1280LT.clearIrqStatus(0xFFFF);               //Clear all the IRQ flags
  SX1280LT.setRx(PERIOBASE_01_MS, 0);            //set no SX1280 RX timeout

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

  RXpacketCount++;
  len = SX1280LT.readPacket(RXBUFFER, RXBUFFER_SIZE);         //read the actual packet, maximum size specified in RXBUFFER_SIZE

  if (len == 0)
  {
    Serial.print(F("Buffer Error"));
  }
  else
  {
    //Serial.print(F("PacketOK,"));
    SX1280LT.printASCIIPacket(RXBUFFER, len);                 //len same as RXPacketL
  }

  Serial.print(",RSSI,");

  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(errors);

  IRQStatus = SX1280LT.readIrqStatus();
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
}


void packet_is_Error()
{
  uint16_t IRQStatus;

  if (ENABLEBUZZER)
  {
    delay(50);
    digitalWrite(BUZZER, HIGH);
    delay(50);
    digitalWrite(BUZZER, LOW);
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


void PrintPacket()
{
  Serial.write(RXBUFFER, RXPacketL);                  //print whole packet, ASCII assumed
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
  SX1280LT.setStandby(STDBY_RC);
  SX1280LT.setRegulatorMode(USE_LDO);
  SX1280LT.setPacketType(PACKET_TYPE_LORA);
  SX1280LT.setRfFrequency(Frequency, 0);
  SX1280LT.setBufferBaseAddress(0, 0);
  SX1280LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  SX1280LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, RXBUFFER_SIZE, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT), 0, 0);
}


void setup(void)
{
  pinMode(LED1, OUTPUT);
  led_Flash(2, 250);

  Serial.begin(Serial_Monitor_Baud);
  Serial.println();
  
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(programversion));
  Serial.println();
  Serial.println();
  Serial.println(F("SX1280LT_LoRa_Simple_RX Starting"));
  Serial.println();

  if (BUZZER >= 0)
  {
    ENABLEBUZZER = true;
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);
    Serial.println(F("BUZZER Enabled"));
  }
  else
  {
    ENABLEBUZZER = false;
    Serial.println(F("BUZZER Not Enabled"));
  }

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

  setup_LoRa();

  led_Flash(2, 250);

  Serial.print(F("Receiver ready - RXBUFFER_SIZE "));
  Serial.println(RXBUFFER_SIZE);
  Serial.println();
}
