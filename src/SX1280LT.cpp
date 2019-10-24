#include <SX1280LT.h>
#include <SPI.h>

//#define SX1280DEBUG             //enable debug messages


SX1280Class::SX1280Class()
{
  //Anything you need when instantiating your object goes here
}


bool SX1280Class::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3)
{
#ifdef SX1280DEBUG
  Serial.println(F("begin()"));
  Serial.println(F("SX1280Class constructor instantiated successfully"));
  Serial.print(F("NSS "));
  Serial.println(_NSS);
  Serial.print(F("NRESET "));
  Serial.println(_NRESET);
  Serial.print(F("RFBUSY "));
  Serial.println(_RFBUSY);
  Serial.print(F("DIO1 "));
  Serial.println(_DIO1);
  Serial.print(F("DIO2 "));
  Serial.println(_DIO2);
  Serial.print(F("DIO3 "));
  Serial.println(_DIO3);
#endif

  pinInit(pinNSS, pinNRESET, pinRFBUSY, pinDIO1, pinDIO2, pinDIO3);

  //assign the passed pins to the class private variable
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = pinDIO2;
  _DIO3 = pinDIO3;


  resetDevice();
  if (checkDevice())
  {
    return true;
  }

  return false;
}


void SX1280Class::pinInit(int8_t _NSS, int8_t _NRESET, int8_t _RFBUSY, int8_t _DIO1, int8_t _DIO2, int8_t _DIO3)
{

#ifdef SX1280DEBUG
  Serial.println(F("Pin_Init()"));
#endif

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);

  if (_DIO1 >= 0)
  {
    pinMode( _DIO1, INPUT);
  }
  else
  {
    //Serial.println(F("DIO1 not used"));
  }

  if (_DIO2 >= 0)
  {
    pinMode( _DIO2, INPUT);
  }
  else
  {
    //Serial.println(F("DIO2 not used"));
  }

  if (_DIO3 >= 0)
  {
    pinMode( _DIO3, INPUT);
  }
  else
  {
    //Serial.println(F("DIO3 not used"));
  }
}



void SX1280Class::spiInit(uint8_t msborder, uint8_t clockdiv, uint8_t mode)
{
#ifdef SX1280DEBUG
  Serial.println(F("spiInit()"));
#endif
  SPI.begin();
  SPI.setBitOrder(msborder);           //depends on SX1280 spi timing
  SPI.setClockDivider(clockdiv);       //too fast may cause error
  SPI.setDataMode(mode);
}


void SX1280Class::rxtxInit(int8_t pinRXEN, int8_t pinTXEN)
{
#ifdef SX1280DEBUG
  Serial.println(F("rxtxpinInit()"));
#endif
   _rxtxpinmode = true;
  _RXEN = pinRXEN;
  _TXEN = pinTXEN;

  pinMode(pinRXEN, OUTPUT);
  digitalWrite(pinRXEN, LOW);           //pins needed for E28-2G4M20S
  pinMode(pinTXEN, OUTPUT);
  digitalWrite(pinTXEN, LOW);           //pins needed for E28-2G4M20S
}


void SX1280Class::setupLoRaRX(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3)
{
  setStandby(MODE_STDBY_RC);
  setRegulatorMode(USE_LDO);
  setPacketType(PACKET_TYPE_LORA);
  setRfFrequency(frequency, offset);
  setBufferBaseAddress(0, 0);
  setModulationParams(modParam1, modParam2, modParam3);
  setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT), 0, 0);
}


void SX1280Class::setupLoRaTX(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3)
{
  setStandby(MODE_STDBY_RC);
  setRegulatorMode(USE_LDO);
  setPacketType(PACKET_TYPE_LORA);
  setRfFrequency(frequency, offset);
  setBufferBaseAddress(0, 0);
  setModulationParams(modParam1, modParam2, modParam3);
  setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
}


void SX1280Class::resetDevice()
{
#ifdef SX1280DEBUG
  Serial.println(F("resetDevice()"));
#endif

  //timings taken from Semtech library
  delay(20);
  digitalWrite(_NRESET, LOW);
  delay(50);
  digitalWrite(_NRESET, HIGH);
  delay(20);
}


void SX1280Class::wake()
{
digitalWrite(_NSS, LOW);
delay(1);
digitalWrite(_NSS, HIGH);
delay(1);
}


void SX1280Class::setStandby(uint8_t standbyconfig)
{
#ifdef SX1280DEBUG
  Serial.println(F("setStandby()"));
#endif

  uint8_t Opcode = 0x80;

  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(Opcode);
  SPI.transfer(standbyconfig);
  digitalWrite(_NSS, HIGH);
  checkBusy();
}


void SX1280Class::checkBusy()
{
#ifdef SX1280DEBUG
  Serial.println(F("checkBusy()"));
#endif

  uint8_t busy_timeout_cnt;
  busy_timeout_cnt = 0;

  while (digitalRead(_RFBUSY))
  {
    delay(1);
    busy_timeout_cnt++;

    if (busy_timeout_cnt > 5)                     //wait 5mS for busy to complete
    {
      Serial.println(F("ERROR - Busy Timeout!"));
      setStandby(0);                              //0:STDBY_RC; 1:STDBY_XOSC
      resetDevice();                             
      config();                                   //re-run saved config
      break;
    }
  }
}


void SX1280Class::setPacketType(uint8_t packettype )
{
#ifdef SX1280DEBUG
  Serial.println(F("setPacketType()"));
#endif
  savedPacketType = packettype;

  writeCommand(RADIO_SET_PACKETTYPE, &packettype, 1);
}


void SX1280Class::writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
#ifdef SX1280DEBUG
  Serial.print(F("writeCommand "));
  Serial.println(Opcode, HEX);
#endif

  uint8_t index;
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(Opcode);

  for (index = 0; index < size; index++)
  {
    SPI.transfer(buffer[index]);
  }
  digitalWrite(_NSS, HIGH);

  if (Opcode != RADIO_SET_SLEEP)
  {
    checkBusy();
  }
}


void SX1280Class::printASCIIPacket(uint8_t *buffer, uint8_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("printASCIIPacket()"));
#endif

  uint8_t index;

  for (index = 0; index < size; index++)
  {
  Serial.write(buffer[index]);
  }

}


void SX1280Class::printHEXPacket(uint8_t *buffer, uint8_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("printHEXPacket()"));
#endif

  uint8_t index;

  for (index = 0; index < size; index++)
  {
    Serial.print(F("["));
    Serial.print(index);
    Serial.print(F("],"));
    Serial.print(buffer[index], HEX);
    Serial.print(F("  "));
  }
}


void SX1280Class::setRegulatorMode(uint8_t mode)
{
#ifdef SX1280DEBUG
  Serial.println(F("setRegulatorMode()"));
#endif

  savedRegulatorMode = mode;

  writeCommand(RADIO_SET_REGULATORMODE, &mode, 1);
}


void SX1280Class::printPacketStatusAllFLRC()
{
#ifdef SX1280DEBUG
  Serial.println(F("printPacketStatusAllFLRC()"));
#endif

  uint8_t status[5];
  readCommand(RADIO_GET_PACKETSTATUS, status, 5);

  Serial.print(F(",PS0,"));
  Serial.print(status[0], HEX);
  Serial.print(F(",PS1,"));
  Serial.print(status[1], HEX);
  Serial.print(F(",PS2,"));
  Serial.print(status[2], HEX);
  Serial.print(F(",PS3,"));
  Serial.print(status[3], HEX);
  Serial.print(F(",PS4,"));
  Serial.print(status[4], HEX);
  printPacketStatus2(status[2]);
}


void SX1280Class::setSyncWord1(uint32_t syncword)
{
#ifdef SX1280DEBUG
  Serial.println(F("setSyncWord1()"));
#endif

  // For FLRC packet type, the SyncWord is one byte shorter and
  // the base address is shifted by one byte
  writeRegister( REG_FLRCSYNCWORD1_BASEADDR, ( syncword >> 24 ) & 0x000000FF );
  writeRegister( REG_FLRCSYNCWORD1_BASEADDR + 1, ( syncword >> 16 ) & 0x000000FF );
  writeRegister( REG_FLRCSYNCWORD1_BASEADDR + 2, ( syncword >> 8 ) & 0x000000FF );
  writeRegister( REG_FLRCSYNCWORD1_BASEADDR + 3, syncword & 0x000000FF );
}


void SX1280Class::printPacketStatus3(uint8_t pstatus)
{
#ifdef SX1280DEBUG
  Serial.println(F("printPacketStatus3()"));
#endif

  //0x0001
  if (pstatus & PktSent)
  {
    Serial.print(F(",PktSent"));
  }

  //0x0008
  if (pstatus & rxpiderr)
  {
    Serial.print(F(",rxpiderr"));
  }

  //0x0010
  if (pstatus & rx_no_ack)
  {
    Serial.print(F(",rx_no_ack"));
  }

}


void SX1280Class::printPacketStatus2(uint8_t pstatus)
{
#ifdef SX1280DEBUG
  Serial.println(F("printPacketStatus2()"));
#endif

  //0x0001
  if (pstatus & PacketCtrlBusy)
  {
    Serial.print(F(",PacketCtrlBusy"));
  }

  //0x0002
  if (pstatus & PacketReceived)
  {
    Serial.print(F(",PacketReceived"));
  }

  //0x0004
  if (pstatus & HeaderReceived)
  {
    Serial.print(F(",HeaderReceived"));
  }

  //0x0008
  if (pstatus & AbortError)
  {
    Serial.print(F(",AbortError"));
  }

  //0x0010
  if (pstatus & CrcError)
  {
    Serial.print(F(",CrcError"));
  }

  //0x0020
  if (pstatus & LengthError)
  {
    Serial.print(F(",LengthError"));
  }

  //0x0040
  if (pstatus & SyncError)
  {
    Serial.print(F(",SyncError"));
  }

  //0x0080
  if (pstatus & Reserved)
  {
    Serial.print(F(",Reserved"));
  }
}


void SX1280Class::printIrqStatus()
{
#ifdef SX1280DEBUG
  Serial.println(F("printIrqStatus()"));
#endif

  uint16_t _IrqStatus;
  _IrqStatus = readIrqStatus();

  //0x0001
  if (_IrqStatus & IRQ_TX_DONE)
  {
    Serial.print(F(",IRQ_TX_DONE"));
  }

  //0x0002
  if (_IrqStatus & IRQ_RX_DONE)
  {
    Serial.print(F(",IRQ_RX_DONE"));
  }

  //0x0004
  if (_IrqStatus & IRQ_SYNCWORD_VALID)
  {
    Serial.print(F(",IRQ_SYNCWORD_VALID"));
  }

  //0x0008
  if (_IrqStatus & IRQ_SYNCWORD_ERROR)
  {
    Serial.print(F(",IRQ_SYNCWORD_ERROR"));
  }

  //0x0010
  if (_IrqStatus & IRQ_HEADER_VALID)
  {
    Serial.print(F(",IRQ_HEADER_VALID"));
  }

  //0x0020
  if (_IrqStatus & IRQ_HEADER_ERROR)
  {
    Serial.print(F(",IRQ_HEADER_ERROR"));
  }

  //0x0040
  if (_IrqStatus & IRQ_CRC_ERROR)
  {
    Serial.print(F(",IRQ_CRC_ERROR"));
  }

  //0x0080
  if (_IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
  {
    Serial.print(F(",IRQ_RANGING_SLAVE_RESPONSE_DONE"));
  }

  //0x0100
  if (_IrqStatus & IRQ_RANGING_SLAVE_REQUEST_DISCARDED)
  {
    Serial.print(",IRQ_RANGING_SLAVE_REQUEST_DISCARDED");
  }

  //0x0200
  if (_IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
  {
    Serial.print(F(",IRQ_RANGING_MASTER_RESULT_VALID"));
  }

  //0x0400
  if (_IrqStatus & IRQ_RANGING_MASTER_RESULT_TIMEOUT)
  {
    Serial.print(F(",IRQ_RANGING_MASTER_RESULT_TIMEOUT"));
  }

  //0x0800
  if (_IrqStatus & IRQ_RANGING_SLAVE_REQUEST_VALID)
  {
    Serial.print(F(",IRQ_RANGING_SLAVE_REQUEST_VALID"));
  }

  //0x1000
  if (_IrqStatus & IRQ_CAD_DONE)
  {
    Serial.print(F(",IRQ_CAD_DONE"));
  }

  //0x2000
  if (_IrqStatus & IRQ_CAD_ACTIVITY_DETECTED)
  {
    Serial.print(F(",IRQ_CAD_ACTIVITY_DETECTED"));
  }

  //0x4000
  if (_IrqStatus & IRQ_RX_TX_TIMEOUT)
  {
    Serial.print(F(",IRQ_RX_TX_TIMEOUT"));
  }

  //0x8000
  if (_IrqStatus & IRQ_PREAMBLE_DETECTED)
  {
    Serial.print(F(",IRQ_PREAMBLE_DETECTED"));
  }
}


void SX1280Class::printRegisters(uint16_t Start, uint16_t End)
{
  //prints the contents of SX1280 registers to serial monitor

#ifdef SX1280DEBUG
  Serial.println(F("printRegisters()"));
#endif

  uint16_t Loopv1, Loopv2, RegData;

  Serial.print(F("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();

  for (Loopv1 = Start; Loopv1 <= End;)           //32 lines
  {
    Serial.print(F("0x"));
    Serial.print((Loopv1), HEX);                 //print the register number
    Serial.print(F("  "));
    for (Loopv2 = 0; Loopv2 <= 15; Loopv2++)
    {
      RegData = readRegister(Loopv1);
      if (RegData < 0x10)
      {
        Serial.print(F("0"));
      }
      Serial.print(RegData, HEX);                //print the register number
      Serial.print(F(" "));
      Loopv1++;
    }
    Serial.println();
  }
}


bool SX1280Class::checkDevice()
{
  //check there is a device out there, writes a register and reads back
#ifdef SX1280DEBUG
  Serial.println(F("checkDevice()"));
#endif

  uint8_t Regdata1, Regdata2;
  Regdata1 = readRegister(0x0907);               //mid byte of frequency setting
  writeRegister(0x0907, (Regdata1 + 1));
  Regdata2 = readRegister(0x0907);               //read changed value back
  writeRegister(0x0907, Regdata1);               //restore register to original value

  if (Regdata2 == (Regdata1 + 1))
  {
    return true;
  }
  else
  {
    return false;
  }
}


void SX1280Class::printHEXByte(uint8_t temp)
{
  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


void SX1280Class::printHEXByte0x(uint8_t temp)
{
  //print a byte, adding 0x
  Serial.print(F("0x"));
  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


void SX1280Class::printASCIIorHEX(uint8_t temp)
{
  if ((temp < 0x10) || (temp > 0x7E))
  {
    Serial.print(F(" ("));
    printHEXByte(temp);
    Serial.print(F(") "));
  }
  else
  {
    Serial.write(temp);
  }
}


void SX1280Class::printAddressInfo()
{
  //print the information for packet last received
#ifdef SX1280DEBUG
  Serial.println(F("printAddressInfo()"));
#endif
  Serial.print(F("RXType,"));
  printASCIIorHEX(_RXPacketType);
  Serial.print(F(",Destination,"));
  printASCIIorHEX(_RXDestination);
  Serial.print(F(",Source,"));
  printASCIIorHEX(_RXSource);
}


void SX1280Class::printReceptionInfoLoRa()
{
  //print the information for packet last received
  //note, _PacketSNR has already been converted into a signed value
  //_PacketRSSI is a signed value also
#ifdef SX1280DEBUG
  Serial.println(F("printReceptionInfoLoRa()"));
#endif

  Serial.print(F("SNR,"));
  Serial.print(_PacketSNR);
  Serial.print(F("dB"));

  Serial.print(F(",RSSI,"));
  Serial.print(_PacketRSSI);
  Serial.print(F("dBm"));
}


bool SX1280Class::readPacketCRCError()
{

#ifdef SX1280DEBUG
  Serial.println(F("readPacketCRCError()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_CRC_ERROR)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX1280Class::readPacketHeaderValid()
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacketHeaderValid()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_HEADER_VALID)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX1280Class::readPacketHeaderError()
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacketHeaderError()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_HEADER_ERROR)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX1280Class::readRXDone()
{
#ifdef SX1280DEBUG
  Serial.println(F("readRXDone()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_RX_DONE)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX1280Class::readTXDone()
{
#ifdef SX1280DEBUG
  Serial.println(F("readTXDone()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_TX_DONE)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void SX1280Class::readRXBufferStatus()
{
#ifdef SX1280DEBUG
  Serial.println(F("readRXBufferStatus()"));
#endif

  uint8_t buffer[2];
  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
}


uint8_t SX1280Class::readPacketAddressedLoRa(uint8_t *RXbuffer, uint8_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacketAddressed()"));
#endif

  uint8_t buffer[2];
  uint8_t index, RegData, RXStart, RXEnd;

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);

  _RXPacketL = buffer[0];
  RXStart = buffer[1];

  if (_RXPacketL > size)                 //check passed buffer is big enough for packet
  {
  _RXPacketL = size;                     //truncate packet if not enough space
  }
   
  RXEnd = _RXPacketL + RXStart;          //calculate the end of the packet in the buffer

  digitalWrite(_NSS, LOW);               //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(RXStart);
  SPI.transfer(0xFF);

  _RXPacketType = SPI.transfer(0);
  _RXDestination = SPI.transfer(0);
  _RXSource = SPI.transfer(0);
  RXStart = RXStart + 3;                 //move start pointer 3 forward 
   
   
  //now fill RXBUFF
  for (index = RXStart; index < RXEnd; index++)
  {
    RegData = SPI.transfer(0);
    RXbuffer[index] = RegData;
  }
  digitalWrite(_NSS, HIGH);
  checkBusy();
  return _RXPacketL;
}



uint8_t SX1280Class::readPacketAddressedFLRC(uint8_t *RXbuffer, uint8_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacketAddressed()"));
#endif

  uint8_t buffer[2];
  uint8_t index, RegData, RXStart, RXEnd;

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);

  _RXPacketL = buffer[0];
  RXStart = buffer[1];
  
  if (_RXPacketL > size)                 //check passed buffer is big enough for packet
  {
  _RXPacketL = size;                     //truncate packet if not enough space
  }

  RXEnd = (RXStart + _RXPacketL);        //calculate the end of the packet in the buffer

  digitalWrite(_NSS, LOW);               //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(RXStart);
  SPI.transfer(0xFF);

  _RXPacketType = SPI.transfer(0);
  _RXDestination = SPI.transfer(0);
  _RXSource = SPI.transfer(0);
  RXStart = RXStart + 3;                 //move start pointer 3 forward 

  //now fill RXBUFF
  for (index = RXStart; index < RXEnd; index++)
  {
    RegData = SPI.transfer(0);
    RXbuffer[index] = RegData;
  }
  digitalWrite(_NSS, HIGH);
  checkBusy();
  return _RXPacketL;
}



uint8_t SX1280Class::readPacketLoRa(uint8_t *RXbuffer, uint8_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacket()"));
#endif

  uint8_t index, regdata, RXStart, RXEnd;
  uint8_t buffer[2];

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  RXStart = buffer[1];

  if (_RXPacketL > size)               //check passed buffer is big enough for packet
  {
  _RXPacketL = size;                   //truncate packet if not enough space
  }

  RXEnd = RXStart + _RXPacketL;        //calculate RXEend
  
  digitalWrite(_NSS, LOW);             //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(RXStart);
  SPI.transfer(0xFF);

  for (index = RXStart; index < RXEnd; index++)
  {
    regdata = SPI.transfer(0);
    RXbuffer[index] = regdata;
  }

  digitalWrite(_NSS, HIGH);
  checkBusy();
  return _RXPacketL;                     
}



uint8_t SX1280Class::readPacketLoRaImplicit(uint8_t *RXbuffer, uint8_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacket()"));
#endif

  uint8_t index, regdata, RXStart, RXEnd;
  uint8_t buffer[2];

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = size;
  RXStart = buffer[1];
  
  if (_RXPacketL > size)               //check passed buffer is big enough for packet
  {
  _RXPacketL = size;                   //truncate packet if not enough space
  }
  
  RXEnd = RXStart + _RXPacketL;        //calculate RXEend

  digitalWrite(_NSS, LOW);             //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(RXStart);
  SPI.transfer(0xFF);

  for (index = RXStart; index < RXEnd; index++)
  {
    regdata = SPI.transfer(0);
    RXbuffer[index] = regdata;
  }

  digitalWrite(_NSS, HIGH);
  
  return _RXPacketL;
}


uint8_t SX1280Class::readPacketFLRC(uint8_t *RXbuffer, uint8_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacket()"));
#endif

  uint8_t index, regdata, RXStart, RXEnd;
  uint8_t buffer[2];

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  RXStart = buffer[1];
  
  if (_RXPacketL > size)                 //check passed buffer is big enough for packet
  {
  _RXPacketL = size;                     //truncate packet if not enough space
  }

  RXEnd = RXStart + _RXPacketL;          //calculate rxend

  digitalWrite(_NSS, LOW);               //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(RXStart);
  SPI.transfer(0xFF);

  for (index = RXStart; index < RXEnd; index++)
  {
    regdata = SPI.transfer(0);
    RXbuffer[index] = regdata;
  }

  digitalWrite(_NSS, HIGH);
  return _RXPacketL;                     //so we can check for packet having enough buffer space
}


bool SX1280Class::sendPacketLoRa(uint8_t *txbuffer, uint8_t size, int32_t txtimeoutmS, int8_t txpower, uint8_t _DIO)
{
#ifdef SX1280DEBUG
  Serial.println(F("sendPacketLoRa()"));
#endif
  uint8_t index;
  uint8_t bufferdata;
  
  if (size == 0)
  {
   return false;
  }
  
  setStandby(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);

  for (index = 0; index < size; index++)
  {
    bufferdata = txbuffer[index];
    SPI.transfer(bufferdata);
  }
  
  digitalWrite(_NSS, HIGH);
  checkBusy();
  _TXPacketL = size;  
  writeRegister(REG_LR_PAYLOADLENGTH, size);
  setTxParams(txpower, RAMP_TIME);
  setTx(PERIOBASE_01_MS, txtimeoutmS);                            //this starts the TX

  while (!digitalRead(_DIO));                                     //Wait for DIO to go high

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )                       //check for timeout
  {
    return false;
  }
  else
  {
    return true;
  }
}




bool SX1280Class::sendPacketFLRC(uint8_t *TXbuffer, uint8_t size, int32_t txtimeoutmS, int8_t TXpower, uint8_t _DIO)
{
#ifdef SX1280DEBUG
  Serial.println(F("sendPacketFLRC()"));
#endif
  uint8_t index;
  uint8_t bufferdata;
  
  if (size == 0)
  {
   return false;
  }
  
  if ((size > 127) || (size < 6 ))
  {
    return false;
  }
  
  setStandby(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);

  for (index = 0; index < size; index++)
  {
    bufferdata = TXbuffer[index];
    SPI.transfer(bufferdata);

  }

  digitalWrite(_NSS, HIGH);
  checkBusy();
  _TXPacketL = size;
  setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, size, savedPacketParam6, savedPacketParam7);
  setTxParams(TXpower, RADIO_RAMP_02_US);
  setTx(PERIOBASE_01_MS, txtimeoutmS);                            //this starts the TX

  while (!digitalRead(_DIO));                                     //Wait for DIO to go high

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )                       //check for timeout
  {
    return false;
  }
  else
  {
    return true;
  }
}


bool SX1280Class::sendPacketAddressedLoRa(uint8_t *TXbuffer, uint8_t size, char TXpackettype, char TXdestination, char TXsource, int32_t txtimeoutmS, int8_t TXpower, uint8_t _DIO)
{
#ifdef SX1280DEBUG
  Serial.println(F("sendPacketAddressedLoRa()"));
#endif
  uint8_t index;
  uint8_t txpacketL = 0;
  uint8_t bufferdata;
  
  if (size == 0)
  {
   return false;
  } 
  
  txpacketL = size + 3;
  setStandby(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);
  SPI.transfer(TXpackettype);                     //Write the packet type
  SPI.transfer(TXdestination);                    //Destination node
  SPI.transfer(TXsource);                         //Source node
  txpacketL = 3 + size;                           //we have added 3 header bytes to size

  for (index = 0; index < size; index++)
  {
    bufferdata = TXbuffer[index];
    SPI.transfer(bufferdata);
  }

  digitalWrite(_NSS, HIGH);
  checkBusy();
  _TXPacketL = size;
  writeRegister(REG_LR_PAYLOADLENGTH, txpacketL);
  setTxParams(TXpower, RADIO_RAMP_02_US);
  setTx(PERIOBASE_01_MS, txtimeoutmS);             //this starts the TX

  while (!digitalRead(_DIO));                      //Wait for DIO to go high

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )        //check for timeout
  {
    return false;
  }
  else
  {
    return true;
  }
}


bool SX1280Class::sendPacketAddressedFLRC(uint8_t *TXbuffer, uint8_t size, char TXpackettype, char TXdestination, char TXsource, int32_t txtimeoutmS, int8_t TXPower, uint8_t _DIO)
{
#ifdef SX1280DEBUG
  Serial.println(F("sendPacketAddressedFLRC()"));
#endif
  uint8_t index;
  uint8_t txpacketL = 0;
  uint8_t bufferdata;
  
  if (size == 0)
  {
   return false;
  }

  txpacketL = size + 3;                   //need to check for minimum of 6 max of 127

  if ((txpacketL > 127) || (txpacketL < 6 ))
  {
    return false;
  }

  setStandby(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);
  SPI.transfer(TXpackettype);                     //Write the packet type
  SPI.transfer(TXdestination);                    //Destination node
  SPI.transfer(TXsource);                         //Source node
  txpacketL = 3 + size;                           //we have added 3 header bytes to _size

  for (index = 0; index < size; index++)
  {
    bufferdata = TXbuffer[index];
    SPI.transfer(bufferdata);
  }

  digitalWrite(_NSS, HIGH);
  checkBusy();
  _TXPacketL = size;
  setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, txpacketL, savedPacketParam6, savedPacketParam7);
  setTxParams(TXPower, RADIO_RAMP_02_US);
  setTx(PERIOBASE_01_MS, txtimeoutmS);             //this starts the TX
  while (!digitalRead(_DIO));                      //Wait for DIO to go high

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )        //check for timeout
  {
    return false;
  }
  else
  {
    return true;
  }
}


void SX1280Class::rxEnable()
{
 //Enable RX mode on device such as Ebyte E28-2G4M20S which have RX and TX enable pins
#ifdef SX1280DEBUG
  Serial.println(F("rxEnable()"));
#endif

  digitalWrite(_RXEN, HIGH);
  digitalWrite(_TXEN, LOW);
}


void SX1280Class::txEnable()
{
  //Enable RX mode on device such as Ebyte E28-2G4M20S which have RX and TX enable pins
#ifdef SX1280DEBUG
  Serial.println(F("txEnable()"));
#endif

  digitalWrite(_RXEN, LOW);
  digitalWrite(_TXEN, HIGH);
}


void SX1280Class::setRfFrequency(uint32_t frequency, int32_t offset)
{
#ifdef SX1280DEBUG
  Serial.println(F("setRfFrequency()"));
#endif

  savedFrequency = frequency;
  savedOffset = offset;

  frequency = frequency + offset;
  uint8_t buffer[3];
  uint32_t freqtemp = 0;
  freqtemp = ( uint32_t )( (double) frequency / (double)FREQ_STEP);
  buffer[0] = ( uint8_t )( ( freqtemp >> 16 ) & 0xFF );
  buffer[1] = ( uint8_t )( ( freqtemp >> 8 ) & 0xFF );
  buffer[2] = ( uint8_t )( freqtemp & 0xFF );
  writeCommand(RADIO_SET_RFFREQUENCY, buffer, 3);
}


int32_t SX1280Class::getFrequencyErrorRegValue()
{
  int32_t FrequencyError;
  uint32_t regmsb, regmid, reglsb, allreg;
  
  setStandby(MODE_STDBY_XOSC);
  
  regmsb = readRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
  regmsb = regmsb & 0x0F;       //clear bit 20 which is always set
  
  regmid = readRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
  
  reglsb = readRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
  setStandby(MODE_STDBY_RC);

  #ifdef LORADEBUG
  Serial.println();
  Serial.print(F("Registers "));
  Serial.print(regmsb,HEX);
  Serial.print(F(" "));
  Serial.print(regmid,HEX);
  Serial.print(F(" "));
  Serial.println(reglsb,HEX);
  #endif
    
  allreg = (uint32_t) ( regmsb << 16 ) | ( regmid << 8 ) | reglsb;

  if (allreg & 0x80000)
  {
  FrequencyError = (0xFFFFF - allreg) * -1;
  }
  else
  {
  FrequencyError = allreg; 
  }

  return FrequencyError;
}


int32_t SX1280Class::getFrequencyErrorHz()
{
  int32_t error, regvalue;
  uint32_t bandwidth;
  float divider;

  bandwidth = getLoRaBandwidth();                   //gets the last configured
  divider = (float) 1625000 / bandwidth;
  regvalue = getFrequencyErrorRegValue();
  error = (FREQ_ERROR_CORRECTION * regvalue) / divider;

  return error;
}


void SX1280Class::setTxParams(int8_t TXpower, uint8_t RampTime)
{
#ifdef SX1280DEBUG
  Serial.println(F("setTxParams()"));
#endif

  uint8_t buffer[2];

  savedTXPower = TXpower;

  //power register is set to 0 to 31 which is -18dBm to +12dBm
  buffer[0] = (TXpower + 18);
  buffer[1] = (uint8_t)RampTime;
  writeCommand(RADIO_SET_TXPARAMS, buffer, 2);
}


void SX1280Class::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
#ifdef SX1280DEBUG
  Serial.println(F("setBufferBaseAddress()"));
#endif

  uint8_t buffer[2];

  buffer[0] = txBaseAddress;
  buffer[1] = rxBaseAddress;
  writeCommand(RADIO_SET_BUFFERBASEADDRESS, buffer, 2);
}


void SX1280Class::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
#ifdef SX1280DEBUG
  Serial.println(F("setDioIrqParams()"));
#endif

  savedIrqMask = irqMask;
  savedDio1Mask = dio1Mask;
  savedDio2Mask = dio2Mask;
  savedDio3Mask = dio3Mask;

  uint8_t buffer[8];

  buffer[0] = (uint8_t) (irqMask >> 8);
  buffer[1] = (uint8_t) (irqMask & 0xFF);
  buffer[2] = (uint8_t) (dio1Mask >> 8);
  buffer[3] = (uint8_t) (dio1Mask & 0xFF);
  buffer[4] = (uint8_t) (dio2Mask >> 8);
  buffer[5] = (uint8_t) (dio2Mask & 0xFF);
  buffer[6] = (uint8_t) (dio3Mask >> 8);
  buffer[7] = (uint8_t) (dio3Mask & 0xFF);
  writeCommand(RADIO_SET_DIOIRQPARAMS, buffer, 8);
}


void SX1280Class::setHighSensitivity()
{
  //set bits 7,6 of REG_LNA_REGIME
#ifdef SX1280DEBUG
  Serial.println(F("setHighSensitivity()"));
#endif

  writeRegister(REG_LNA_REGIME, (readRegister(REG_LNA_REGIME) | 0xC0));
}


void SX1280Class::setLowPowerRX()
{
  //clear bits 7,6 of REG_LNA_REGIME
#ifdef SX1280DEBUG
  Serial.println(F("setLowPowerRX()"));
#endif

  writeRegister(REG_LNA_REGIME, (readRegister(REG_LNA_REGIME) & 0x3F));
}


void SX1280Class::setRx(uint8_t _periodBase, uint16_t _periodBaseCount)
{
#ifdef SX1280DEBUG
  Serial.println(F("setRx()"));
#endif
  
  if (_rxtxpinmode)  
  {
  rxEnable();
  }
  
  uint8_t buffer[3];
  
  clearIrqStatus(IRQ_RADIO_ALL);                             //clear all interrupt flags
  buffer[0] = _periodBase;
  buffer[1] = ( uint8_t ) ((_periodBaseCount >> 8 ) & 0x00FF);
  buffer[2] = ( uint8_t ) (_periodBaseCount & 0x00FF);
  writeCommand(RADIO_SET_RX, buffer, 3);
}


uint16_t SX1280Class::readIrqStatus()
{
#ifdef SX1280DEBUG
  Serial.print(F("readIrqStatus()"));
#endif

  uint16_t temp;
  uint8_t buffer[2];

  readCommand(RADIO_GET_IRQSTATUS, buffer, 2);
  temp = ((buffer[0] << 8) + buffer[1]);
  return temp;
}


bool SX1280Class::packetOK(uint16_t mask)
{
#ifdef SX1280DEBUG
  Serial.print(F("packetOK()"));
#endif

  uint16_t temp;
  temp = readIrqStatus();

  if (temp == mask)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void SX1280Class::readRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("readRegisters()"));
#endif

  uint16_t index;
  uint8_t addr_l, addr_h;

  addr_h = address >> 8;
  addr_l = address & 0x00FF;
  checkBusy();

#ifdef SX1280DEBUG
  Serial.println(F("ReadRegisters "));
  Serial.print(addr_h, HEX);
  Serial.print(addr_l, HEX);
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_READ_REGISTER);
  SPI.transfer(addr_h);               //MSB
  SPI.transfer(addr_l);               //LSB
  SPI.transfer(0xFF);
  for (index = 0; index < size; index++)
  {
#ifdef SX1280DEBUG
    Serial.println(F(" "));
    Serial.print(*(buffer + index));
#endif
    *(buffer + index) = SPI.transfer(0xFF);
  }
#ifdef SX1280DEBUG
  Serial.println();
#endif
  digitalWrite(_NSS, HIGH);
  checkBusy();
}


uint8_t SX1280Class::readRegister(uint16_t address)
{
#ifdef SX1280DEBUG
  Serial.println(F("readRegister()"));
#endif

  uint8_t data;

  readRegisters(address, &data, 1);
  return data;
}


void SX1280Class::clearIrqStatus(uint16_t irqMask)
{
#ifdef SX1280DEBUG
  Serial.println(F("clearIrqStatus()"));
#endif

  uint8_t buffer[2];

  buffer[0] = (uint8_t) (irqMask >> 8);
  buffer[1] = (uint8_t) (irqMask & 0xFF);
  writeCommand(RADIO_CLR_IRQSTATUS, buffer, 2);
}


void SX1280Class::readCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("readCommand()"));
#endif

  uint8_t i;
  checkBusy();

  digitalWrite(_NSS, LOW);
  SPI.transfer(Opcode);
  SPI.transfer(0xFF);

  for ( i = 0; i < size; i++ )
  {
    *(buffer + i) = SPI.transfer(0xFF);
  }
  digitalWrite(_NSS, HIGH);

  checkBusy();
}


void SX1280Class::readPacketReceptionLoRa()
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacketReceptionLoRa()"));
#endif

  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5) ;
  _PacketRSSI = -status[0] / 2;

  if ( status[1] < 128 )
  {
    _PacketSNR = status[1] / 4 ;
  }
  else
  {
    _PacketSNR = (( status[1] - 256 ) / 4);
  }

}


uint8_t SX1280Class::readPacketStatus3FLRC()
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacketStatus3FLRC()"));
#endif

  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5);
  return status[3];
}


uint8_t SX1280Class::readPacketStatus2FLRC()
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacketStatus2FLRC()"));
#endif

  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5);
  return status[2];
}


void SX1280Class::readPacketReceptionFLRC()
{
#ifdef SX1280DEBUG
  Serial.println(F("readPacketReceptionFLRC()"));
#endif

  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5);
  _PacketRSSI = -status[0] / 2;
}


void SX1280Class::writeRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeRegisters()"));
#endif
  uint8_t addr_l, addr_h;
  uint8_t i;

  addr_l = address & 0xff;
  addr_h = address >> 8;
  checkBusy();

#ifdef SX1280DEBUG
  Serial.println(F("WriteRegisters "));
  Serial.print(addr_h, HEX);
  Serial.print(addr_l, HEX);
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_REGISTER);
  SPI.transfer(addr_h);   //MSB
  SPI.transfer(addr_l);   //LSB

  for (i = 0; i < size; i++)
  {
#ifdef SX1280DEBUG
    Serial.println(F(" "));
    Serial.print(buffer[i]);
#endif
    SPI.transfer(buffer[i]);
  }

#ifdef SX1280DEBUG
  Serial.println();
#endif
  digitalWrite(_NSS, HIGH);
  checkBusy();
}


void SX1280Class::writeRegister(uint16_t address, uint8_t value)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeRegister()"));
#endif

  writeRegisters( address, &value, 1 );
}


void SX1280Class::setSleep(uint8_t sleepconfig)
{
#ifdef SX1280DEBUG
  Serial.println(F("setSleep()"));
#endif
  setStandby(MODE_STDBY_RC);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_SET_SLEEP);
  SPI.transfer(sleepconfig);
  digitalWrite(_NSS, HIGH);
  delay(1);           //allow time for shutdown
}


void SX1280Class::setTx(uint8_t _periodBase, uint16_t _periodBaseCount)
{

#ifdef SX1280DEBUG
  Serial.println(F("setTx()"));
#endif
  
  if (_rxtxpinmode)  
  {
  txEnable();
  }
  
  uint8_t buffer[3];
  
  clearIrqStatus(IRQ_RADIO_ALL);                             //clear all interrupt flags 
  buffer[0] = _periodBase;
  buffer[1] = ( uint8_t )( ( _periodBaseCount >> 8 ) & 0x00FF );
  buffer[2] = ( uint8_t )( _periodBaseCount & 0x00FF );
  writeCommand(RADIO_SET_TX, buffer, 3 );
}


uint8_t SX1280Class::readRXPacketL()
{
#ifdef SX1280DEBUG
  Serial.println(F("readRXPacketL()"));
#endif

  uint8_t buffer[2];

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  return _RXPacketL;
}


uint8_t SX1280Class::readPacketRSSI()
{
  return _PacketRSSI;
}


uint8_t SX1280Class::readPacketSNR()
{
  return _PacketSNR;
}


uint8_t SX1280Class::readRXPacketType()
{
  return _RXPacketType;
}


uint8_t SX1280Class::readRXDestination()
{
  return _RXDestination;
}


uint8_t SX1280Class::readRXSource()
{
  return _RXSource;
}


bool SX1280Class::sendFIFOLoRa(int16_t txtimeoutmS, int8_t txpower, uint8_t _DIO)
{
#ifdef SX1280DEBUG
  Serial.println(F("sendBufferLoRa()"));
#endif

  setPacketParams(savedPacketParam1, savedPacketParam2, _TXPacketL, savedPacketParam4, savedPacketParam5, savedPacketParam6, savedPacketParam7);
  setTxParams(txpower, RAMP_TIME);
  setTx(PERIOBASE_01_MS, txtimeoutmS);                     //this starts the TX

  while (!digitalRead(_DIO));                              //Wait for DIO to go high

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )                //check for timeout
  {
    return false;
  }
  else
  {
    return true;
  }
}


void SX1280Class::setPacketParams(uint8_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5, uint8_t packetParam6, uint8_t packetParam7)
{
#ifdef SX1280DEBUG
  Serial.println(F("SetPacketParams()"));
#endif

  savedPacketParam1 = packetParam1;
  savedPacketParam2 = packetParam2;
  savedPacketParam3 = packetParam3;
  savedPacketParam4 = packetParam4;
  savedPacketParam5 = packetParam5;
  savedPacketParam6 = packetParam6;
  savedPacketParam7 = packetParam7;

  uint8_t buffer[7];
  buffer[0] = packetParam1;
  buffer[1] = packetParam2;
  buffer[2] = packetParam3;
  buffer[3] = packetParam4;
  buffer[4] = packetParam5;
  buffer[5] = packetParam6;
  buffer[6] = packetParam7;
  writeCommand(RADIO_SET_PACKETPARAMS, buffer, 7);

}


void SX1280Class::setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3)
{
#ifdef SX1280DEBUG
  Serial.println(F("setModulationParams_FLRC()"));
#endif

  uint8_t buffer[3];

  savedModParam1 = modParam1;
  savedModParam2 = modParam2;
  savedModParam3 = modParam3;

  buffer[0] = modParam1;
  buffer[1] = modParam2;
  buffer[2] = modParam3;

  writeCommand(RADIO_SET_MODULATIONPARAMS, buffer, 3);
}


bool SX1280Class::config()
{
#ifdef SX1280DEBUG
  Serial.println(F("config()"));
#endif

  resetDevice();
  setStandby(MODE_STDBY_RC);
  setRegulatorMode(savedRegulatorMode);
  setPacketType(savedPacketType);
  setRfFrequency(savedFrequency, savedOffset);
  setModulationParams(savedModParam1, savedModParam2, savedModParam3);
  setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5, savedPacketParam6, savedPacketParam7);
  setDioIrqParams(savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask);       //set for IRQ on RX done on DIO1
  return true;
}


void SX1280Class::printSavedModulationParams()
{
  Serial.print(F("SavedModulationParams "));
  Serial.print(savedModParam1, HEX);
  Serial.print(F(" "));
  Serial.print(savedModParam2, HEX);
  Serial.print(F(" "));
  Serial.print(savedModParam3, HEX);
}


uint8_t SX1280Class::readsavedModParam1()
{
  return savedModParam1;
}


uint8_t SX1280Class::readsavedModParam2()
{
  return savedModParam2;
}


uint8_t SX1280Class::readsavedModParam3()
{
  return savedModParam3;
}


uint8_t SX1280Class::readsavedPower()
{
  return savedTXPower;
}


uint32_t SX1280Class::getFreqInt()
{
  //get the current set device frequency, return as long integer
  uint8_t Msb, Mid, Lsb;
  uint32_t uinttemp;
  float floattemp;
  Msb = readRegister(REG_RFFrequency23_16);
  Mid = readRegister(REG_RFFrequency15_8);
  Lsb = readRegister(REG_RFFrequency7_0);
  floattemp = ((Msb * 0x10000ul) + (Mid * 0x100ul) + Lsb);
  floattemp = ((floattemp * FREQ_STEP) / 1000000ul);
  uinttemp = (uint32_t)(floattemp * 1000000);
  return uinttemp;
}


uint32_t SX1280Class::returnBandwidth(uint8_t data)
{
  switch (data)
  {
    case LORA_BW_0200:
      return 203125;

    case LORA_BW_0400:
      return 406250;

    case LORA_BW_0800:
      return 812500;

    case LORA_BW_1600:
      return 1625000;

    default:
      break;
  }

  return 0x0;                      //so that a bandwidth not set can be identified
}


uint8_t SX1280Class::returnSF(uint8_t data)
{
  return (data >> 4);
}


//*******************************************************************************
//Ranging routines
//*******************************************************************************


void SX1280Class::setRangingRangingAddress(uint32_t address)
{
#ifdef LORADEBUG
  Serial.println(F("SetRangingRangingAddress()"));
#endif

  uint8_t buffer[4];

  buffer[0] = (address >> 24u ) & 0xFFu;
  buffer[1] = (address >> 16u) & 0xFFu;
  buffer[2] = (address >>  8u) & 0xFFu;
  buffer[3] = (address & 0xFFu);

  writeRegisters(0x916, buffer, 4 );
}


void SX1280Class::setRangingRequestAddress(uint32_t address)
{
#ifdef LORADEBUG
  Serial.println(F("SetRangingRangingAddress()"));
#endif

  uint8_t buffer[4];

  buffer[0] = (address >> 24u ) & 0xFFu;
  buffer[1] = (address >> 16u) & 0xFFu;
  buffer[2] = (address >>  8u) & 0xFFu;
  buffer[3] = (address & 0xFFu);

  writeRegisters(0x912, buffer, 4 );
}


void SX1280Class::setRangingCalibration(uint16_t cal)
{
  writeRegister( REG_LR_RANGINGRERXTXDELAYCAL, ( uint8_t )( ( cal >> 8 ) & 0xFF ) );
  writeRegister( REG_LR_RANGINGRERXTXDELAYCAL + 1, ( uint8_t )( ( cal ) & 0xFF ) );
}


void SX1280Class::setRangingRole(uint8_t role)
{
#ifdef LORADEBUG
  Serial.println(F("SetRangingRangingAddress()"));
#endif

  uint8_t buffer[1];

  buffer[0] = role;

  writeCommand(RADIO_SET_RANGING_ROLE, buffer, 1 );
}


double SX1280Class::getRangingResult(uint8_t resultType)
{
  uint32_t valLsb = 0;
  float val = 0.0;

  setStandby(MODE_STDBY_XOSC);
  writeRegister( 0x97F, readRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
  writeRegister( REG_LR_RANGINGRESULTCONFIG, ( readRegister( REG_LR_RANGINGRESULTCONFIG ) & MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
  valLsb = ( ( (uint32_t) readRegister( REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( readRegister( REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( readRegister( REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
  setStandby(MODE_STDBY_RC);

  if (valLsb >= 0x800000)                  //raw reg value at low distance can goto 0x800000 which is negative, set distance to zero if this happens
  {
    valLsb = 0;
  }

  // Conversion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280

  switch (resultType)
  {
    case RANGING_RESULT_RAW:
      // Convert the ranging LSB to distance in meter. The theoretical conversion from register value to distance [m] is given by:
      // distance [m] = ( complement2( register ) * 150 ) / ( 2^12 * bandwidth[MHz] ) ). The API provide BW in [Hz] so the implemented
      // formula is complement2( register ) / bandwidth[Hz] * A, where A = 150 / (2^12 / 1e6) = 36621.09
      val = ( double )complement2( valLsb, 24 ) / ( double ) getLoRaBandwidth( ) * 36621.09375;
      break;

    case RANGING_RESULT_AVERAGED:
    case RANGING_RESULT_DEBIASED:
    case RANGING_RESULT_FILTERED:
      val = ( double )valLsb * 20.0 / 100.0;
      break;
    default:
      val = 0.0;
      break;
  }

  return val;
}


uint32_t SX1280Class::getRangingResultRegValue(uint8_t resultType)
{
  uint32_t valLsb = 0;

  setStandby(MODE_STDBY_XOSC);
  writeRegister( 0x97F, readRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
  writeRegister( REG_LR_RANGINGRESULTCONFIG, ( readRegister( REG_LR_RANGINGRESULTCONFIG ) & MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
  valLsb = ( ( (uint32_t) readRegister( REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( (uint32_t) readRegister( REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( readRegister( REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
  setStandby(MODE_STDBY_RC);
  return valLsb;
}


uint32_t SX1280Class::getLoRaBandwidth()
{
  uint32_t bwValue = 0;

  switch (savedModParam2)
  {
    case LORA_BW_0200:
      bwValue = 203125;
      break;
    case LORA_BW_0400:
      bwValue = 406250;
      break;
    case LORA_BW_0800:
      bwValue = 812500;
      break;
    case LORA_BW_1600:
      bwValue = 1625000;
      break;
    default:
      bwValue = 0;
  }
  return bwValue;
}


int32_t SX1280Class::complement2( uint32_t num, uint8_t bitCnt )
{
    
    int32_t retVal = ( int32_t )num;
    if( num >= 2<<( bitCnt - 2 ) )
    {
        retVal -= 2<<( bitCnt - 1 );
    }
    return retVal;
}


//***************************************************************************
//
//Direct FIFO buffer access routines, these read and write data directly to an from
//the SX1280 buffer. A start address of 0 in the buffer is assumed.
//The startWriteFIFO() routine opens up the SPI interface for direct transfers and
//leaves the bus open. Do not perform other accesses to the SX1280 until the
//endWriteFIFO() function is called.
//
//***************************************************************************


void SX1280Class::startWriteFIFO()
{
#ifdef SX1280DEBUG
  Serial.println(F("startWriteFIFO()"));
#endif

  _TXcount = 0;                            //this variable used to keep track of bytes written
  setStandby(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);
  //SPI interface ready for byte to write to buffer
}


void SX1280Class::endWriteFIFO()
{
#ifdef SX1280DEBUG
  Serial.println(F("endWriteFIFO()"));
#endif
  _TXPacketL = _TXcount;
  digitalWrite(_NSS, HIGH);
  checkBusy();
}


void SX1280Class::startReadFIFO()
{
#ifdef SX1280DEBUG
  Serial.println(F("startReadFIFO()"));
#endif

  uint8_t rxstart;
  uint8_t buffer[2];

  _RXcount = 0;
  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  rxstart = buffer[1];
  checkBusy();
  digitalWrite(_NSS, LOW);                     //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(rxstart);
  SPI.transfer(0xFF);

  //next line would be data = SPI.transfer(0);
  //SPI interface ready for byte to read from
}


void SX1280Class::endReadFIFO()
{
#ifdef SX1280DEBUG
  Serial.println(F("endReadFIFO()"));
#endif

  digitalWrite(_NSS, HIGH);
  checkBusy();
}



void SX1280Class::writeUint8(uint8_t x)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeUint8()"));
#endif

  SPI.transfer(x);

  _TXcount++;                     //increment count of bytes written
}

uint8_t SX1280Class::readUint8()
{
#ifdef SX1280DEBUG
  Serial.println(F("readUint8()"));
#endif
  byte x;

  x = SPI.transfer(0);

  _RXcount++;                      //increment count of bytes read
  return (x);
}


void SX1280Class::writeInt8(int8_t x)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeInt8()"));
#endif

  SPI.transfer(x);

  _TXcount++;                     //increment count of bytes written
}


int8_t SX1280Class::readInt8()
{
#ifdef SX1280DEBUG
  Serial.println(F("readInt8()"));
#endif
  int8_t x;

  x = SPI.transfer(0);

  _RXcount++;                      //increment count of bytes read
  return (x);
}


void SX1280Class::writeInt16(int16_t x)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeInt16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXcount = _TXcount + 2;         //increment count of bytes written
}


int16_t SX1280Class::readInt16()
{
#ifdef SX1280DEBUG
  Serial.println(F("readInt16()"));
#endif
  byte lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXcount = _RXcount + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX1280Class::writeUint16(uint16_t x)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeUint16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXcount = _TXcount + 2;         //increment count of bytes written
}


uint16_t SX1280Class::readUint16()
{
#ifdef SX1280DEBUG
  Serial.println(F("writeUint16()"));
#endif
  byte lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXcount = _RXcount + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX1280Class::writeInt32(int32_t x)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeInt32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    int32_t f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXcount = _TXcount + 4;         //increment count of bytes written
}


int32_t SX1280Class::readInt32()
{
#ifdef SX1280DEBUG
  Serial.println(F("readInt32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    int32_t f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXcount = _RXcount + 4;         //increment count of bytes read
  return readdata.f;
}


void SX1280Class::writeUint32(uint32_t x)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeUint32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    uint32_t f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXcount = _TXcount + 4;         //increment count of bytes written
}


uint32_t SX1280Class::readUint32()
{
#ifdef SX1280DEBUG
  Serial.println(F("readUint32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    uint32_t f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXcount = _RXcount + 4;         //increment count of bytes read
  return readdata.f;
}


void SX1280Class::writeFloat(float x)
{
#ifdef SX1280DEBUG
  Serial.println(F("writeFloat()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    float f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXcount = _TXcount + 4;         //increment count of bytes written
}


float SX1280Class::readFloat()
{
#ifdef SX1280DEBUG
  Serial.println(F("readFloat()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    float f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXcount = _RXcount + 4;         //increment count of bytes read
  return readdata.f;
}

