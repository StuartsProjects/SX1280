The Examples folder contains a series of test programs for the SX1280. The programs are described at the bottom of the page. Its recommended to use the **SX1280_LoRa_Register_Test** program first to test the device connections are correct. 

A common feature of the programs is the uses of a series of defines in the settings.h file. These defines determine the frequency used, the bandwidth the spreading factor and coding rate used, for example;


    #define Frequency 2445000000        //frequency of transmissions    
    #define Bandwidth LORA_BW_0400      //LoRa bandwidth    
    #define SpreadingFactor LORA_SF10   //LoRa spreading factor    
    #define CodeRate LORA_CR_4_5        //LoRa coding rate


The full list of options you can set are;

    //LoRa bandwidths
    #define LORA_BW_0200  //203khz
    #define LORA_BW_0400  //406khz
    #define LORA_BW_0800  //812khz
    #define LORA_BW_1600  //1625khz

.

    ////LoRa spreading factors
    #define LORA_SF5
    #define LORA_SF6 
    #define LORA_SF7
    #define LORA_SF8
    #define LORA_SF9
    #define LORA_SF10 
    #define LORA_SF11
    #define LORA_SF12
.

    //LoRa coding rates
    #define LORA_CR_4_5
    #define LORA_CR_4_6
    #define LORA_CR_4_7
    #define LORA_CR_4_8



**SX1280_LoRa_Register_Test** - Checks to see if a SX1280 device can be found. prints a list of register read. 

**SX1280LT_LoRa_Simple_TX** - Transmits a series of LoRa packets according to the LoRa parameters in the settings.h file. Used together with matching RX program

**SX1280LT_LoRa_Simple_RX** - Receives a LoRa packets according to the LoRa parameters in the settings.h file. Results displayed in IDE serial monitor. Used together wit matching TX program.