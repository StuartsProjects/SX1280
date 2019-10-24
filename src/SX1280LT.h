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
  DONE - Tidy Order of routines
  DONE - Check that send packet routines are sendPacketAddressedLoRa sendPacketAddressedFLRC
  DONE - Remove need for setPacketParamsFLRC( and setPacketParamsLoRa 
  DONE - Remove need for setModulationParamsFLRC, setModulationParamsLoRa
  DONE - Is _SavedLoRaBandwidth needed ? - No
  DONE - Why readpacket and readPacketLoRa - Just in case differences in packet treatmentr in future
  DONE - Check for bool sendPacket - Returns status of send
  DONE - sendPacketAddressedLoRa/FLRC and sendPacketLoRa/FLRC - is it necessary to trap zero length packets ? - Yes
  DONE - Does printASCIIPacket(uint8_t *buff, uint8_t tsize) need size--; - No
  DONE - printASCIIPacket( printing one byte too many ? - No prints null character present at end of buffer
  DONE - Chenage GetFreqInt() to getFreqInt();
  DONE - SendPacketAddressedLoRa remove startmS, timemS
  DONE - Change sendBufferLoRa to sendFIFOLoRa
  DONE - Add packet implicit mode support - Added readPacketLoRaImplicit and examples
  REJECTED - Remove need for <SX1280LT_Includes.h> - cannot see the need
  DONE - Move spiInit out of class instance constructor
  DONE - Check for packet reception overwriting TXbuffer and RXbuffer ends
  DONE - Check changes to detect _RXPacketL > buffer size are replicated in implicit reception
  DONE - Remove definitions for TXBUFFER_SIZE, RXBUFFER_SIZE
  REJECTED - Check readPacketAddressed(RXBUFFER, RXBUFFER_SIZE) for packet length 256 - packet limited to 255bytes
  REJECTED - Remove need for readPacketReceptionLoRa() - cannot as packet reception flagged outside of library
  DONE - Check for uint32_t txtimeout - all changed from unit16_t to uint32_t
  DONE - Change all TXtimeout, txbuffer, rx buffer in function calls to TX and RX
  DONE - Check if printPacketStatus2 printPacketStatus3 are FLRC only - not currently used anyway
  DONE - Check Whitening for FLRC #define   WHITENING   0x08 - removed, use RADIO_WHITENING_OFF
  DONE - Save settings to allow for a SX1280_Config recovery from busy timeout error
  DONE - Change function of setSleep() to setSleep(uint8_t config);
  DONE - Check setSleep(Retain_Data_RAM) retains register comments in sleep - Yes, setSleep(Retain_None) retains nothing
  DONE - Check setSleep puts device in low current mode, additional sleep current of SX1280 about 0.1uA in register retention mode
  DONE - Check recovery from busy timeout error
  DONE - Check frequency error output
  DONE - Check validity of offset calc to setfrequency
  DONE - Add frequency offset in setFrequency to all examples
  DONE - Check that checkBusy() only used in functions where there is SPI access
  DONE - Put  clearIrqStatus(IRQ_RADIO_ALL) in setTX() and SetRX()
  DONE - Set all examples to SF7 and LORA_BW_0400 - apart from ranging which is SF10
  DONE - Test all example programs
  DONE - Remove SX1280DEBUG2 defines
  DONE - Check RX and TX enable pin functions
  DONE - Remove start_power apart from TX link tester
  
  Review ranging operation
  Why is SX1280LT.setPacketType(PACKET_TYPE_LORA) required before getFreqInt works (example 1)
  Review error rate in FLRC mode
  Ranging requestor 0dBm in packet receipt mode ?  
  Question the frequency error corection in the data sheet, is it really 1600/bandwithkhz ?  
  
**************************************************************************/

class SX1280Class  {
  public:
    SX1280Class();

    bool begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3);
    void pinInit(int8_t _NSS, int8_t _NRESET, int8_t _RFBUSY, int8_t _DIO1, int8_t _DIO2, int8_t _DIO3);
    void spiInit(uint8_t MSBOrder, uint8_t ClockDiv, uint8_t Mode);
    void resetDevice();
    void setStandby(uint8_t StdbyConfig);
    void checkBusy();
    void setSleep(uint8_t sleepconfig); 
    void setRegulatorMode(uint8_t mode);
    bool checkDevice();
    bool config();
    void wake();

    void setupLoRaRX(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3);
    void setupLoRaTX(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3);
    
    void writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size );
    void readCommand( uint8_t Opcode, uint8_t *buffer, uint16_t size );  
    void writeRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    void writeRegister( uint16_t address, uint8_t value );
    void readRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    uint8_t readRegister( uint16_t address );
    void printRegisters(uint16_t Start, uint16_t End);
    void setPacketParams(uint8_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5, uint8_t packetParam6, uint8_t packetParam7);
    void setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3);
    void printSavedModulationParams();

    void setRfFrequency( uint32_t frequency, int32_t offset );
    uint32_t getFreqInt();
    int32_t getFrequencyErrorRegValue();
    int32_t getFrequencyErrorHz();
    void setTxParams(int8_t TXpower, uint8_t RampTime);
    void setTx(uint8_t _periodBase, uint16_t _periodBaseCount);
    bool readTXDone();
    void setRx(uint8_t _periodBase, uint16_t _periodBaseCount);
    void setLowPowerRX();
    void setHighSensitivity();
    bool readRXDone();
    void readRXBufferStatus();
    uint8_t readRXPacketL();
    uint8_t readPacketRSSI();
    uint8_t readPacketSNR();

    void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void setPacketType(uint8_t PacketType);

    void clearIrqStatus( uint16_t irq );
    uint16_t readIrqStatus();
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
    void printIrqStatus();

    bool readPacketCRCError();
    bool readPacketHeaderValid();
    bool readPacketHeaderError();

    uint32_t returnBandwidth(uint8_t data);
    uint8_t returnSF(uint8_t data);
    uint8_t readsavedPower();

    void printASCIIorHEX(uint8_t temp);
    void printHEXByte(uint8_t temp);
    void printHEXByte0x(uint8_t temp);
    bool packetOK(uint16_t mask);
    void printASCIIPacket(uint8_t *buff, uint8_t tsize);
    void printHEXPacket(uint8_t *buff, uint8_t tsize);

    uint8_t readsavedModParam1();
    uint8_t readsavedModParam2();
    uint8_t readsavedModParam3();

    //*******************************************************************************
    //Packet addressing routines
    //*******************************************************************************

    uint8_t readRXPacketType();
    uint8_t readRXDestination();
    uint8_t readRXSource();
    void printAddressInfo();

    //*******************************************************************************
    //LoRa specific Routines
    //*******************************************************************************

    bool sendPacketAddressedLoRa(uint8_t *txbuffer, uint8_t size, char txpackettype, char txdestination, char txsource, int32_t TXtimeoutmS, int8_t TXpower, uint8_t _DIO);
    bool sendPacketLoRa(uint8_t *txbuffer, uint8_t size, int32_t txtimeoutmS, int8_t txpower, uint8_t _DIO);
    uint8_t readPacketAddressedLoRa(uint8_t *rxbuffer, uint8_t size);
    uint8_t readPacketLoRa(uint8_t *rxbuffer, uint8_t _size);
    uint8_t readPacketLoRaImplicit(uint8_t *rxbuffer, uint8_t size);
    void readPacketReceptionLoRa();
    void printReceptionInfoLoRa();
    bool sendFIFOLoRa(int16_t txtimeoutmS, int8_t txpower, uint8_t _DIO);
    uint32_t getLoRaBandwidth();

    //*******************************************************************************
    //FLRC specific Routines
    //*******************************************************************************

    bool sendPacketFLRC(uint8_t *txbuffer, uint8_t size, int32_t txtimeoutmS, int8_t txpower, uint8_t _DIO);
    bool sendPacketAddressedFLRC(uint8_t *txbuffer, uint8_t size, char txPacketType, char txDestination, char txSource, int32_t TXTimeoutmS, int8_t TXPower, uint8_t DIO);
    uint8_t readPacketAddressedFLRC(uint8_t *rxbuffer, uint8_t size);
    uint8_t readPacketFLRC(uint8_t *rxbuffer, uint8_t _size);
    void readPacketReceptionFLRC();
    uint8_t readPacketStatus3FLRC();
    uint8_t readPacketStatus2FLRC();
    void setSyncWord1( uint32_t syncword);
    void printPacketStatusAllFLRC();
    void printPacketStatus2(uint8_t pstatus);
    void printPacketStatus3(uint8_t pstatus);


    //*******************************************************************************
    //Read Write Buffer commands
    //*******************************************************************************

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

    //*******************************************************************************
    //Ranging routines
    //*******************************************************************************

    void setRangingRangingAddress(uint32_t address);
    void setRangingRequestAddress(uint32_t address);
    void setRangingRole(uint8_t role);
    void setRangingCalibration(uint16_t cal);
    double getRangingResult(uint8_t resultType);
    int32_t complement2( const uint32_t num, const uint8_t bitCnt );
    uint32_t getRangingResultRegValue(uint8_t resultType);

    //*******************************************************************************
    //RXTX Switch routines
    //*******************************************************************************

    void rxtxInit(int8_t RXEN, int8_t TXEN);
    void rxEnable();
    void txEnable();

  private:

    int8_t _NSS, _NRESET, _RFBUSY, _DIO1, _DIO2, _DIO3;
    int8_t _RXEN, _TXEN;
    uint8_t _RXPacketL;             //length of packet received
    uint8_t _RXPacketType;          //type number of received packet
    uint8_t _RXDestination;         //destination address of received packet
    uint8_t _RXSource;              //source address of received packet
    int8_t  _PacketRSSI;            //RSSI of received packet
    int8_t  _PacketSNR;             //signal to noise ratio of received packet
    int8_t  _TXPacketL;             //transmitted packet length
    uint8_t _RXcount;               //used to keep track of the bytes read from SX1280 buffer during readFloat() etc
    uint8_t _TXcount;               //used to keep track of the bytes written to SX1280 buffer during writeFloat() etc
    bool _rxtxpinmode = false;      //set to true if RX and TX pin mode is used.

    //config variables are 36 bytes, allows for device to be reset and reconfigured via config();
    uint8_t  savedRegulatorMode;
    uint8_t  savedPacketType;
    uint32_t savedFrequency, savedOffset;
    uint8_t  savedModParam1, savedModParam2, savedModParam3;
    uint8_t  savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5, savedPacketParam6, savedPacketParam7;
    uint16_t savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask;
    int8_t   savedTXPower;

};
#endif
