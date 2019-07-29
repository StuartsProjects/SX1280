#ifndef SX1280LT_h
#define SX1280LT_h

#include "Arduino.h"
#include <SX1280LT_Includes.h>

/**************************************************************************

  ToDO

  DONE - Check packet length setting in send packet routines sendPacketAddressedLoRa
  DONE - Check packet length setting in send packet routines sendPacketAddressedFLRC
  DONE - Check packet length setting in send packet routines sendPacketLoRa
  DONE - Check packet length setting in send packet routines sendPacketFLRC
  Save settings to allow for a SX1280_Config recovery from busy timeout error
  Remove need for setPacketParamsFLRC( and setPacketParamsLoRa(
  Remove need for setModulationParamsFLRC, setModulationParamsLoRa
  Is _SavedLoRaBandwidth needed ?
  Does printASCIIPacket(uint8_t *buff, uint8_t tsize) need size--;
  Check Whitening for FLRC #define   WHITENING   0x08          //No Whitnening avaialble in FLRC, must be set to off
  printASCIIPacket( printing one byte too many ?
  sendPacketAddressedLoRa( needs to trap zero length packet

**************************************************************************/

class SX1280Class  {
  public:
    //Constructor
    SX1280Class();

    //Methods
    bool begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3);
    void pinInit(int8_t _NSS, int8_t _NRESET, int8_t _RFBUSY, int8_t _DIO1, int8_t _DIO2, int8_t _DIO3);
    void spiInit(uint8_t MSBOrder, uint8_t ClockDiv, uint8_t Mode);
    void rxtxpinInit(int8_t RXEN, int8_t TXEN);
    void resetDevice();
    void setStandby(uint8_t StdbyConfig);
    void checkBusy();
    void setPacketType(uint8_t PacketType);
    void writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size );

    void setRfFrequency( uint32_t frequency, int32_t offset );
    void setTxParams(int8_t TXpower, uint8_t RampTime);
    void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void setRx(uint8_t _periodBase, uint16_t _periodBaseCount);
    void readCommand( uint8_t Opcode, uint8_t *buffer, uint16_t size );
    void readRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    uint8_t readRegister( uint16_t address );
    void clearIrqStatus( uint16_t irq );
    void readPacketReceptionLoRa();
    void writeRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    void writeRegister( uint16_t address, uint8_t value );
    void setSleep();
    void setTx(uint8_t _periodBase, uint16_t _periodBaseCount);
    uint16_t readIrqStatus();
    void setRegulatorMode(uint8_t mode);

    //added by LoRaTracker
    void rxEnable(int8_t _RXEN, int8_t _TXEN);
    void txEnable(int8_t _RXEN, int8_t _TXEN);
    uint32_t sendPacketAddressedLoRa(uint8_t *txbuffer, uint8_t size, char txpackettype, char txdestination, char txsource, int16_t txtimeoutmS, int8_t txpower, uint8_t _DIO);
    bool sendPacketLoRa(uint8_t *TXbuffer, uint8_t size, int16_t TXTimeoutmS, int8_t TXPower, uint8_t _DIO);
    uint8_t readPacketAddressed(uint8_t *RXbuffer, uint8_t size);
    uint8_t readPacket(uint8_t *_RXbuffer, uint8_t _size);
    bool readPacketCRCError();
    bool readPacketHeaderValid();
    bool readPacketHeaderError();
    void printReceptionInfoLoRa();
    void printAddressInfo();
    void printASCIIorHEX(uint8_t temp);
    void printHEXByte(uint8_t temp);
    void printHEXByte0x(uint8_t temp);
    bool packetOK(uint16_t mask);
    bool checkDevice();
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
    void printRegisters(uint16_t Start, uint16_t End);
    void printIrqStatus();
    void setHighSensitivity();
    void setLowPowerRX();
    void printASCIIPacket(uint8_t *buff, uint8_t tsize);
    void printHEXPacket(uint8_t *buff, uint8_t tsize);
    bool readRXDone();
    bool readTXDone();
    void readRXBufferStatus();

    //added by LoRaTracker for FLRC
    bool sendPacketFLRC(uint8_t *_TXbuffer, uint8_t _size, int16_t _TXTimeoutmS, int8_t _TXPower, uint8_t _DIO);
    bool sendPacketAddressedFLRC(uint8_t *_TXbuffer, uint8_t _size, char _TXPacketType, char _TXDestination, char _TXSource, int16_t _TXTimeoutmS, int8_t _TXPower, uint8_t _DIO);
    void readPacketReceptionFLRC();
    void printPacketStatusAllFLRC();
    uint8_t readPacketStatus3FLRC();
    uint8_t readPacketStatus2FLRC();
    void printPacketStatus2(uint8_t pstatus);
    void printPacketStatus3(uint8_t pstatus);
    void setSyncWord1( uint32_t syncword);
    uint8_t readRXPacketL();
    uint8_t readPacketRSSI();
    uint8_t readPacketSNR();
    uint8_t readRXPacketType();
    uint8_t readRXDestination();
    uint8_t readRXSource();
    uint8_t readsavedModParam1();
    uint8_t readsavedModParam2();
    uint8_t readsavedModParam3();

    void startWriteFIFO();
    void endWriteFIFO();
    void startReadFIFO();
    void endReadFIFO();

    void writeUint8(uint8_t x);
    uint8_t readUint8();

    void writeInt8(int8_t x);
    int8_t readInt8();

    void writeInt16(int16_t x);
    int16_t readInt16();

    void writeUint16(uint16_t x);
    uint16_t readUint16();

    void writeInt32(int32_t x);
    int32_t readInt32();

    void writeUint32(uint32_t x);
    uint32_t readUint32();

    void writeFloat(float x);
    float readFloat();

    

    uint32_t GetFreqInt();
    uint16_t returnBandwidth(uint8_t data);
    uint8_t returnSF(uint8_t data);
    uint8_t readsavedPower();

    bool sendBufferLoRa(int16_t txtimeoutmS, int8_t txpower, uint8_t _DIO);
    void setPacketParams(uint8_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5, uint8_t packetParam6, uint8_t packetParam7);
    void setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3);
    bool config();
    void printSavedModulationParams();

    //*******************************************************************************
    //Ranging routines
    //*******************************************************************************

    //ranging specific routines
    void setRangingRangingAddress(uint32_t address);
    void setRangingRequestAddress(uint32_t address);
    void setRangingRole(uint8_t role);
    void setRangingCalibration(uint16_t cal);
    int32_t getLoRaBandwidth();
    double getRangingResult(uint8_t resultType);
    int32_t complement2( const uint32_t num, const uint8_t bitCnt );
    uint32_t getRangingResultRegValue(uint8_t resultType);

  private:

    int8_t _NSS, _NRESET, _RFBUSY, _DIO1, _DIO2, _DIO3;
    int8_t _RXEN, _TXEN;
    uint8_t _SavedLoRaBandwidth;   //used to record the LoRa bandwidth configured, needed for FLRC
    uint8_t _RXPacketL;             //length of packet received
    uint8_t _RXPacketType;          //type number of received packet
    uint8_t _RXDestination;         //destination address of received packet
    uint8_t _RXSource;              //source address of received packet
    int8_t  _PacketRSSI;            //RSSI of received packet
    int8_t  _PacketSNR;             //signal to noise ratio of received packet
    int8_t  _TXPacketL;
    uint8_t _RXcount;               //used to keep track of the bytes read from SX1280 buffer during readFloat() etc
    uint8_t _TXcount;               //used to keep track of the bytes written to SX1280 buffer during writeFloat() etc


    //config variables 36 bytes, allows for device to be reset and reconfigured via confg();
    uint8_t  savedRegulatorMode;
    uint8_t  savedPacketType;
    uint32_t savedFrequency, savedOffset;
    uint8_t  savedModParam1, savedModParam2, savedModParam3;
    uint8_t  savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5, savedPacketParam6, savedPacketParam7;
    uint16_t savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask;
    int8_t   savedTXPower;


};
#endif
