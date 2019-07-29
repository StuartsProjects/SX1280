#define programversion "V1.0"
#define Serial_Monitor_Baud 115200

#include "Settings.h"
#include <SPI.h>
#include "SX1280LT.h"

SX1280Class SX1280LT;

#ifdef ENABLEDISPLAY
#include <Wire.h>
#include <LiquidCrystal_I2C.h>                                   //www.4tronix.co.uk/arduino/sketches/LiquidCrystal_V1.2.1.zip
//address of PCF8574 can change, I have noted addresses of 0x27 and 0x3F
LiquidCrystal_I2C disp(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  //Set the LCD I2C address and pins used
#endif

uint16_t rangeing_error_count, rangeing_valid_count, rangeing_valid_results;
uint16_t IrqStatus, Calibration;
uint32_t endwaitmS, startrangingmS, range_result, range_result_sum, range_result_average;
float distance, distance_sum, distance_average, distance_average_adjusted;
bool ranging_error;


void loop()
{
  uint8_t index;
  distance_sum = 0;
  range_result_sum = 0;
  rangeing_valid_count = 0;
  Calibration = CalibrationSF10BW400;

  setup_RangingTX();

  for (index = 1; index <= range_count; index++)
  {

    if (reset_device)
    {
      Serial.print("ResetDevice,");
      SX1280LT.resetDevice();
      setup_RangingTX();
    }

    Serial.print(RangingTXPower);
    Serial.print("dBm");
    SX1280LT.clearIrqStatus(0xFFFF);                                 //Clear all the IRQ flags
    startrangingmS = millis();
    SX1280LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
    SX1280LT.setTx(PERIOBASE_01_MS, rangingTXTimeoutmS);             //this sends the ranging packet

    endwaitmS = millis() + waittimemS;

    while (!(digitalRead(DIO1)) && (millis() < endwaitmS));          //wait for Ranging valid or timeout

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
        range_result_sum = range_result_sum + range_result;
        Serial.print(",RAW,");
        Serial.print(range_result, HEX);
        distance = SX1280LT.getRangingResult(RANGING_RESULT_AVERAGED);
        distance_sum = distance_sum + distance;
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
  
  rangeing_valid_results++;
  range_result_average = (range_result_sum / rangeing_valid_count);
  distance_average = (distance_sum / rangeing_valid_count);
  distance_average_adjusted = (distance_average * distance_adjustment);
  
  Serial.println();
  Serial.print("Average Range Result ");
  Serial.print(range_result_average, HEX);
  Serial.println();
  Serial.print("Average Distance ");
  Serial.print(distance_average, 1);
  Serial.print("m");
  Serial.println();
  Serial.print("Adjusted average Distance ");
  Serial.print(distance_average_adjusted, 1);
  Serial.print("m");
  Serial.println();
  Serial.println();
  
  #ifdef ENABLEDISPLAY
  display_screen1();
  #endif
  
}


#ifdef ENABLEDISPLAY
void display_screen1()
{
  disp.clear();
  disp.setCursor(0,0);
  disp.print("Count ");
  disp.print(rangeing_valid_results);
  disp.print("  Err ");
  disp.print(rangeing_error_count);
  disp.setCursor(0,1);
  disp.print("Raw Result ");
  disp.print(range_result_average, HEX);
  disp.setCursor(0,2);
  disp.print("Distance ");
  disp.print(distance_average,1);
  disp.print("m");
  disp.setCursor(0,3);
  disp.print("Adjusted ");
  disp.print(distance_average_adjusted,1);
  disp.print("m");
}
#endif


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void setup_RangingTX()
{
  SX1280LT.setStandby(STDBY_RC);
  SX1280LT.setPacketType(PACKET_TYPE_RANGING);
  SX1280LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  SX1280LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  SX1280LT.setRfFrequency(Frequency, Offset);
  SX1280LT.setTxParams(RangingTXPower, RADIO_RAMP_02_US);
  SX1280LT.setRangingRequestAddress(TXaddress);
  SX1280LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RANGING_MASTER_RESULT_VALID + IRQ_RANGING_MASTER_RESULT_TIMEOUT), 0, 0);              //set for IRQ on RX done
  SX1280LT.setRangingCalibration(CalibrationSF10BW400);
  SX1280LT.setRangingRole(RADIO_RANGING_ROLE_MASTER);
  SX1280LT.setHighSensitivity();                    //set high sensitivity RX mode
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
  Serial.println("SX1280LT_Ranging_Requester Starting");

  pinMode(LED1, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  led_Flash(2, 250);

  Serial.println("Checking device");

  if (SX1280LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3))
  {
    Serial.println(F("Device found"));
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                            //long fast flash indicates device error
    }
  }

#ifdef ENABLEDISPLAY
  Serial.println("Display Enabled");
  Wire.begin();
  disp.begin(20, 4);                   //initialize the lcd for 20 chars 4 lines, turn on backlight
  disp.backlight();                    //backlight on
  disp.setCursor(0, 0);
  disp.print("Ranging Ready");
#endif



}

