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

  The LoRa settings to use are specified in the 'Settings.h' file. The program sends short test packet
  starting at start_power and ending at end_power (see Settings.h file). The packet sent contains the
  power used to send the packet. The SX1280LT_LoRa_Simple_RX program will print the received packet
  out like this;

  T*1+011,RSSI,-47dBm,SNR,10dB,Length,7,Packets,14,Errors,0,IRQreg,8012
  T*1 001,RSSI,-49dBm,SNR,7dB,Length,7,Packets,15,Errors,0,IRQreg,8012
  T*1-011,RSSI,-50dBm,SNR,10dB,Length,7,Packets,16,Errors,0,IRQreg,8012

  Above shows 3 packets received, the first at +01dBm (+011 in printout), the second at 0dBm (001 in
  printout) and the third at -1dBm (-011) in printout.


******************************************************************************************************
*/

#define programversion "V1.0"
#define Serial_Monitor_Baud 115200

#include <SPI.h>
#include "Settings.h"
#include <SX1280LT.h>
#include <SX1280LT_Program_Definitions.h>

SX1280Class SX1280LT;

boolean SendOK;
int8_t TestPower;
uint8_t TXPacketL;

void loop()
{
  Serial.print("Send Start Test Sequence");

  Send_Test1Mode_Packet();                        //send initial packet, contains batch number in case used by RX
  Serial.println();

  for (TestPower = start_power; TestPower >= end_power; TestPower--)
  {
    digitalWrite(LED1, HIGH);
    Serial.print(TestPower);
    Serial.print("dBm ");
    Serial.print("Test Packet> ");
    Serial.flush();
    Send_Test_Packet(TestPower);
    Serial.println();
    delay((packet_delay / 2));
    digitalWrite(LED1, LOW);
    delay((packet_delay / 2));
  }

  Serial.println("Finished Test Sequence");
  Serial.println();
}


void packet_is_OK()
{
  uint16_t IRQStatus;
  IRQStatus = SX1280LT.readIrqStatus();                    //get the IRQ status
  Serial.print(F(" "));
  Serial.print(TXPacketL);
  Serial.print(F(" Bytes SentOK"));
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  SX1280LT.printIrqStatus();
}


void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = SX1280LT.readIrqStatus();                    //get the IRQ status
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  SX1280LT.printIrqStatus();
  delay(packet_delay);                                     //change LED flash so packet error visible
  delay(packet_delay);
  digitalWrite(LED1, HIGH);
  delay(packet_delay);
  delay(packet_delay);
  digitalWrite(LED1, LOW);
}


void Send_Test_Packet(int8_t lpower)
{
  //build and send the test packet in addressed form, 3 bytes will be added to begining of packet
  int8_t temppower;
  uint8_t buff[3];                                         //link test payload is 4 bytes
  TXPacketL = sizeof(buff);
  if (lpower < 0)
  {
    buff[0] = '-';
  }
  else
  {
    buff[0] = '+';
  }

  if (TestPower == 0)
  {
    buff[0] = ' ';
  }

  temppower = TestPower;

  if (temppower < 0)
  {
    temppower = -temppower;
  }

  if (temppower > 9)
  {
    buff[1] = '1';
    buff[2] = ((temppower - 10) + 0x30);
  }
  else
  {
    buff[1] = '0';
    buff[2] = (temppower + 0x30);
  }

  SX1280LT.printASCIIPacket(buff, sizeof(buff));

  SendOK = SX1280LT.sendPacketAddressedLoRa(buff, sizeof(buff), Testpacket, Broadcast, '1', 1000, TestPower, DIO1);

  if (!SendOK)
  {
    packet_is_Error();
  }
  else
  {
    packet_is_OK();
  }
}


void Send_Test1Mode_Packet()
{
  //useed to allow an RX to switch mode and print totals

  uint8_t buff[3];          //Test1Mode_Packet is 3 bytes

  buff[0] = '1';
  buff[1] = lowByte(Batch);
  buff[2] = highByte(Batch);
  TXPacketL = sizeof(buff);

  SendOK = SX1280LT.sendPacketAddressedLoRa(buff, sizeof(buff), TestMode1, Broadcast, ThisNode, 1000, start_power, DIO1);

  if (!SendOK)
  {
    packet_is_Error();
  }
  else
  {
    packet_is_OK();
  }

  delay(mode_delaymS);
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
  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);

  Serial.begin(Serial_Monitor_Baud);
  Serial.println();
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println();

  Serial.println(F("4_SX1280LT_LoRa_Link_Test_TX Starting"));

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

  Serial.println(F("LoRa Transmitter ready"));
  Serial.println();
}

