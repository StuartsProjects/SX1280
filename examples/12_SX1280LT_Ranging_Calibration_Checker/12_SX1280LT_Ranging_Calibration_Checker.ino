/*
******************************************************************************************************

  lora Programs for Arduino

  Copyright of the author Stuart Robinson 24/10/19

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  This program is a calibration checker for the ranging function of the SX1280. It uses an initial calibration
  value of circa 9500 and sends a ranging request. The distance is estimated and printed out. The calibartion
  value is incremented and the cycle repeats. As the calibration value increases the reported distance between
  the nodes will decrease till it becomes close to zero. When this happens that calibration is appropriate for
  use in the range measuring programs. The ranging calibration value is only valid for a badndwidth of 400khz
  and a spreading factor of 10.

  Changes:

  To Do:

******************************************************************************************************
*/

#define programversion "V1.0"
#define Serial_Monitor_Baud 115200

#include "Settings.h"

#include <SPI.h>

#include "SX1280LT.h"
SX1280Class SX1280LT;

uint8_t rangeing_error_count, rangeing_valid_count;
uint16_t timeoutErrors, IrqStatus, Calibration;
uint32_t endwaitmS, startrangingmS, range_result;
float distance, total_distance, adjusted_distance;
boolean ranging_error;


void loop()
{
  uint16_t index;
  total_distance = 0;
  rangeing_error_count = 0;
  rangeing_valid_count = 0;

  for (index = CalibrationStart; index <= CalibrationEnd; index = index + 10)
  {
    Calibration = index;

    if (reset_device)
    {
      Serial.print("ResetDevice,");
      SX1280LT.resetDevice();
    }

    setup_RangingTX(Calibration);
    Serial.print(RangingTXPower);
    Serial.print("dBm");
    startrangingmS = millis();
    SX1280LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
    SX1280LT.setTx(PERIOBASE_01_MS, rangingTXTimeoutmS);              //this sends the ranging packet

    endwaitmS = millis() + waittimemS;

    while (!(digitalRead(DIO1)) && (millis() < endwaitmS));           //wait for Ranging valid or timeout

    if (millis() > endwaitmS)
    {
      Serial.print(",Timout");
      rangeing_error_count++;

      range_result = 0;

    }
    else
    {
      IrqStatus = SX1280LT.readIrqStatus();

      if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
      {
        rangeing_valid_count++;
        digitalWrite(BUZZER, HIGH);
        ranging_error = false;
        digitalWrite(LED1, HIGH);
        Serial.print(",Valid");
        Serial.print(",Irq,");
        Serial.print(IrqStatus, HEX);
        range_result = SX1280LT.getRangingResultRegValue(RANGING_RESULT_AVERAGED);

        Serial.print(",RAW,");
        Serial.print(range_result, HEX);
        distance = SX1280LT.getRangingResult(RANGING_RESULT_AVERAGED);
        total_distance = total_distance + distance;
        Serial.print(",Calibration,");
        Serial.print(Calibration);
        Serial.print(",Distance,");
        Serial.print(distance, 1);
        Serial.print("m");
        Serial.print(",Time,");
        Serial.print(millis() - startrangingmS);
        Serial.print("mS");
        Serial.print(",Valid,");
        Serial.print(rangeing_valid_count);
        Serial.print(",Errors,");
        Serial.print(rangeing_error_count);

        delay(100);                              //extra delay if no display

        digitalWrite(BUZZER, LOW);
        digitalWrite(LED1, LOW);
      }
      else
      {
        ranging_error = true;
        rangeing_error_count++;
        distance = 0;
        range_result = 0;
        Serial.print(",NotValid");
        Serial.print(",Irq,");
        Serial.print(IrqStatus, HEX);
      }
    }
    Serial.println();
    delay(packet_delaymS);
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


void setup_RangingTX(uint16_t lCalibration)
{
  SX1280LT.setStandby(MODE_STDBY_RC);
  SX1280LT.setPacketType(PACKET_TYPE_RANGING);
  SX1280LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  SX1280LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  SX1280LT.setRfFrequency(Frequency, Offset);
  SX1280LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
  SX1280LT.setRangingRequestAddress(TXaddress);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RANGING_MASTER_RESULT_VALID + IRQ_RANGING_MASTER_RESULT_TIMEOUT), 0, 0);              //set for IRQ on RX done
  SX1280LT.setRangingCalibration(lCalibration);
  SX1280LT.setRangingRole(RADIO_RANGING_ROLE_MASTER);
  SX1280LT.setHighSensitivity();
}


void setup()
{
  Serial.begin(Serial_Monitor_Baud);            //setup Serial console ouput
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Compiled "));
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println(F("Stuart Robinson"));
  Serial.println();

  Serial.println("12_SX1280LT_Ranging_Calibration_Checker Starting");

  pinMode(LED1, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  led_Flash(2, 125);

  Serial.println("Checking device");

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

