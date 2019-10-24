//Program_Definitions.h

/*
*******************************************************************************************************************************
  Easy Build LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 22/10/19

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
  of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  These are general program definitions used by various LoRaTracker programs. They are not needed if your writing
  your own applications.

  To Do:

*******************************************************************************************************************************
*/


/*
******************************************************************************************************
  Definitions for packet types
******************************************************************************************************
*/

const char ACK = 'A';                   //Acknowledge
const char AFC = 'a';                   //AFC packet 
const char LinkReport = 'B';            //HAB Style in CSV ASCII format
const char bLinkReport = 'b';           //binary style link report
const char ClearToSend = 'C';           
const char ClearToSendCommand = 'c';
const char Error = 'E';                 
const char NoFix = 'F';                 //GPS no fix  
const char NoGPS = 'G';                 //No GPS found    
const char GLONASSDetected = 'g';       //Glonass output from GPS detected   
const char IsLost = 'H';                //packet to notify of lost status (Help)
const char Info = 'i';                  //binary info packet
const char PowerUp = 'P';               //sent on tracker start 
const char Memory = 'M';                //Memory write command  
const char NACK = 'N';                  //Not Acknowledge, rejected
const char NACKCommand = 'n';           //Command not accepted
const char Repeated = 'R';              //Marks packet as repeated 
const char Ranging = 'r';               //Ranging request
const char LocationCSVPacket = 'S';     //Short loacation packet in CSV ASCII format
const char LocationBinaryPacket = 's';  //Short location packet in binary format
const char Testpacket = 'T';            //Test packet
const char Test = 'T';                  //Test
const char Timepacket = 't';            //seconds up time
const char Wakeup = 'W';                //Wakeup from snooze
const char ResetTracker = 'X';          //Reset tracker   
const char Config1 = 'Y';               //Change config1 byte
const char Config0 = 'Z';               //Change config0 byte 

const char Bind = '#';                  //Bind packet
const char Sensor1 = '!';               //Sensor packet1
const char Sensor2 = '"';               //Sensor packet2
const char WritePacketMemory = '0';     //Write bytes to Memory
const char TestMode1 = '1';             //used to switch to Testmode1 settings
const char TestMode2 = '2';             //used to switch to Testmode2 settings
const char INCFreq = '3';               //Increase frequency 
const char DECFreq = '4';               //Decrease frequency
const char InfoRelay = '5';             //used to relay screen SNR and RSSI in ping test. 
const char LMLCSVPacket = '8';          //short LML payload; lat,lon,alt in CSV ASCII format
const char LMLCSVPacket_Repeated = '9'; //short LML payload that has been repeated

const char Broadcast = '*';             //Broadcast address
const char PacketStart = '$';           //Command packets have a payload of at least one byte, so put this at start
const char HABPacket = '$';             //HAB Style location payload in CSV ASCII format

const byte Strip = 1;                   //value for setting stip of addressing information on packet
const byte NoStrip = 0;                 //value for sending addressing information as part of packet

/*
******************************************************************************************************
  Bit numbers for current_config byte settings  end definitions for packet types
******************************************************************************************************
*/

//Bit numbers for current_config byte settings in transmitter (addr_Default_config1)
const byte SearchEnable = 0;           //bit num to set in config byte to enable search mode packet
const byte TXEnable = 1;               //bit num to set in config byte to enable transmissions
const byte FSKRTTYEnable = 2;          //bit num to set in config byte to enable FSK RTTY
const byte CheckFence = 3;             //bit number to control whether fence is checked 0 = NoCheck
const byte DozeEnable = 4;             //bit num to set in config byte to put tracker in Doze mode
const byte AddressStrip = 6;           //bit number to control address part of payload, 1 to strip, 0 to allow
const byte GPSHotFix = 7;              //bit when set enables GPS Hot Fix mode.

//Bit numbers for current_config byte settings in receiver ((addr_Default_config2)
const byte UseSD = 0;                  //bit num to set in config byte to enable search mode packet
const byte TXEnable_RX = 1;            //bit num to set in config byte to enable transmissions
const byte Send_AFSK_RTTY_Upload = 2;  //bit num to set in config byte to enable AFSK_RTTY send
const byte Send_Bluetooth_Uplink = 3;  //bit num to set in config byte to enable Bluetooth uplink
const byte Do_MENUDEBUG = 4;           //bit num to set in config byte to enable Menu debugging


//Status byte settings
const byte GPSFix = 0;                 //flag bit number to indicate GPS has current fix
const byte UBLOXDynamicModel6Set = 1;  //flag bit set when UBLOX dynamic mode 6 has been set (and Checked)
const byte GLONASSisoutput = 2;        //flag bit number to indicate GLONASS found
const byte GPSError = 3;               //flag bit to indicate GPS error or some sort.
const byte TrackerLost = 6;            //flag bit indication that tracker in lost mode
const byte NoGPSTestMode = 7;          //flag bit number to indicate tracker in in GPS test mode


#define SwitchOn 1                     //for tracker GPS control
#define DontSwitch 0
#define LeaveOff 0
#define LeaveOn 1
#define Enabled 1
#define Disabled 0

#define DoGPSPowerSwitch 1              //used to make GPS code clearer 
#define NoGPSPowerSwitch 0              //used to make GPS code clearer

/*
******************************************************************************************************
  Addressing information for variables stored in Memory
  This is kept to 1024 bytes of storage.
  First 256 bytes is program configuration data and settings
  Second 256 bytes is for storing changing data such as sequence number and reset count.
  Third 256 bytes is for storing the last received HAB data packet.
  Forth 256 bytes is for storing the location and time of local GPS fix and tracker GPS fix.
  The bind data is from addr_StartBindData to addr_EndBindData, no more than 128bytes.
  ******************************************************************************************************
*/

/*********************************************************************
  START bind data
**********************************************************************/
const unsigned int addr_StartMemory = 0x00;                   //the start of memory
const unsigned int addr_StartBindData = 0x00;                 //the start of the Bind data
const unsigned int addr_TrackerMode_Frequency = 0x00;         //unsigned long 4 bytes
const unsigned int addr_SearchMode_Frequency = 0x04;          //unsigned long 4 bytes
const unsigned int addr_CommandMode_Frequency = 0x08;         //unsigned long 4 bytes
const unsigned int addr_TrackerMode_Bandwidth = 0x0C;         //byte 1 byte
const unsigned int addr_SearchMode_Bandwidth = 0x0D;          //byte 1 byte
const unsigned int addr_CommandMode_Bandwidth = 0x0E;         //byte 1 byte
const unsigned int addr_TrackerMode_SpreadingFactor = 0x0F;   //byte 1 byte
const unsigned int addr_SearchMode_SpreadingFactor = 0x10;    //byte 1 byte
const unsigned int addr_CommandMode_SpreadingFactor = 0x11;   //byte 1 byte
const unsigned int addr_TrackerMode_CodeRate = 0x12;          //byte 1 byte
const unsigned int addr_SearchMode_CodeRate = 0x13;           //byte 1 byte
const unsigned int addr_CommandMode_CodeRate = 0x14;          //byte 1 byte
const unsigned int addr_TrackerMode_Power = 0x15;             //byte 1 byte
const unsigned int addr_SearchMode_Power = 0x16;              //byte 1 byte
const unsigned int addr_CommandMode_Power = 0x17;             //byte 1 byte
const unsigned int addr_RangingMode_Frequency = 0x18;         //unsigned long 4 bytes
const unsigned int addr_RangingMode_Bandwidth = 0x1C;         //byte 1 byte
const unsigned int addr_RangingMode_SpreadingFactor = 0x1D;   //byte 1 byte
const unsigned int addr_RangingMode_CodeRate = 0x1E;          //byte 1 byte
const unsigned int addr_RangingMode_Power = 0x1F;             //byte 1 byte
const unsigned int addr_EndBindData = 0x1F;                   //marks the end of the Bind data
/*********************************************************************
  END bind data
**********************************************************************/

/*********************************************************************
  START config data
**********************************************************************/
const unsigned int addr_StartConfigData = 0x20;               //the start of config data in memory
const unsigned int addr_west_fence = 0x20;                    //float 4 bytes
const unsigned int addr_east_fence = 0x24;                    //float 4 bytes
const unsigned int addr_south_fence = 0x28;                   //float 4 bytes
const unsigned int addr_north_fence = 0x2C;                   //float 4 bytes
const unsigned int addr_FlightID = 0x30;                      //Character array 16 bytes max
const unsigned int addr_Default_config2 = 0x40;               //byte 1 byte for receiver 
const unsigned int addr_promiscuous_Mode = 0x41;              //byte 1 byte
const unsigned int addr_Default_config1 = 0x42;               //byte 1 byte 
const unsigned int addr_ThisNode = 0x43;                      //char 1 byte
const unsigned int addr_RemoteControlNode = 0x44;             //char 1 byte
const unsigned int addr_ControlledNode = 0x45;                //char 1 byte
const unsigned int addr_key0 = 0x46;                          //char 1 byte
const unsigned int addr_key1 = 0x47;                          //char 1 byte
const unsigned int addr_key2 = 0x48;                          //char 1 byte
const unsigned int addr_key3 = 0x49;                          //char 1 byte

const unsigned int addr_BindCRC = 0x50;                       //the 16 bit CRC of the current Bind Data, addr_StartBindData to addr_EndBindData
const unsigned int addr_CalibrationOffset = 0x80;             //int 2 bytes
const unsigned int addr_kelvin_offset = 0x82;                 //unsigned int 2 bytes
const unsigned int addr_conversion_slope = 0x84;              //float 4 bytes
const unsigned int addr_adc_constant = 0x88;                  //unsigned long int 4 bytes
const unsigned int addr_FSKRTTYbaudDelay = 0x8C;              //unsigned int 2 bytes
const unsigned int addr_Sleepsecs  = 0x90;                    //unsigned int 2 bytes
const unsigned int addr_WaitGPSFixSeconds = 0x92;             //unsigned int 2 bytes
const unsigned int addr_FSKRTTYRegshift = 0x94;               //unsigned int 2 bytes
const unsigned int addr_FSKRTTYleadin = 0x96;                 //unsigned int 2 bytes
const unsigned int addr_MinsToLost = 0x98;                    //unsigned int 2 bytes
const unsigned int addr_TXDelaySecs = 0x9A;                   //unsigned int 2 bytes
const unsigned int addr_TXLostDelaySecs = 0x9C;               //unsigned int 2 bytes (not used)
const unsigned int addr_pulseerrorlimit = 0x9E;               //unsigned int 2 bytes (not used)
const unsigned int addr_holddifference = 0xA0;                //unsigned int 2 bytes
const unsigned int addr_RCpulseshort = 0xA2;                  //unsigned int 2 bytes
const unsigned int addr_RCpulselong = 0xA4;                   //unsigned int 2 bytes
const unsigned int addr_GPSerrorLimit = 0xA6;                 //unsigned int 2 bytes
const unsigned int addr_inc_on_error = 0xA8;                  //char 1 byte
const unsigned int addr_dec_on_OK = 0xA9;                     //char 1 byte
const unsigned int addr_FSKRTTYpips = 0xAA;                   //char 1 byte
const unsigned int addr_Default_config3 = 0xAB;               //byte 1 byte
const unsigned int addr_AFSKRTTYbaudDelay = 0xAC;             //unsigned int 2 bytes
const unsigned int addr_Default_config4 = 0xAD;               //byte 1byte
const unsigned int addr_Cmd_WaitSecs = 0xAE;                  //byte 1byte

const unsigned int addr_TestLocation_page0 = 0xFF;            //used as a location for read\write tests
const unsigned int addr_EndConfigData = 0xFF;


/*********************************************************************
  END config data
**********************************************************************/

/*********************************************************************
  START Program data
**********************************************************************/
const unsigned int addr_StartProgramData = 0x100;             //the start of program data in memory
const unsigned int addr_ResetCount = 0x100;                   //unsigned long int 4 bytes
const unsigned int addr_SequenceNum = 0x104;                  //unsigned long int 4 bytes
const unsigned int addr_mASecs = 0x108;                       //unsigned long int 4 bytes
const unsigned int addr_TRStatus = 0x10C;                     //byte, used to store status
const unsigned int addr_TestLocation_page1 = 0x1FF;           //used as a location for read\write tests
const unsigned int addr_EndProgramData = 0x110;               //the end of program data in memory
/*********************************************************************
  END Program data
**********************************************************************/

/*********************************************************************
  START HAB Payload data
**********************************************************************/
const unsigned int addr_StartHABPayloadData = 0x200;          //for storing last received HAB tracker packet
const unsigned int addr_TestLocation_page2 = 0x2FF;           //used as a location for read\write tests
const unsigned int addr_EndHABPayloadData = 0x2FF;            //store up to 256 bytes of HAB Packet
/*
********************************************************************
  END HAB Payload data
*********************************************************************
*/

/*********************************************************************
  START GPS CoordinateData
**********************************************************************/
//for storing last received GPS co-ordinates from local and remote tracker GPS
const unsigned int addr_StartCoordinateData = 0x300;
const unsigned int addr_TRLat = 0x300;                        //float 4 bytes
const unsigned int addr_TRLon = 0x304;                        //float 4 bytes
const unsigned int addr_TRAlt = 0x308;                        //unsigned int 2 bytes
const unsigned int addr_TRHour = 0x30C;                       //byte 1 byte;  Note times for last tracker co-ordinates come from local GPS time
const unsigned int addr_TRMin = 0x310;                        //byte 1 byte
const unsigned int addr_TRSec = 0x311;                        //byte 1 byte
const unsigned int addr_TRDay = 0x312;                        //byte 1 byte
const unsigned int addr_TRMonth = 0x313;                      //byte 1 byte
const unsigned int addr_TRYear = 0x314;                       //byte 1 byte
const unsigned int addr_LocalLat = 0x318;                     //float 4 bytes
const unsigned int addr_LocalLon = 0x31C;                     //float 4 bytes
const unsigned int addr_LocalAlt = 0x320;                     //unsigned int 2 bytes
const unsigned int addr_LocalHour = 0x322;                    //byte 1 byte
const unsigned int addr_LocalMin = 0x323;                     //byte 1 byte
const unsigned int addr_LocalSec = 0x324;                     //byte 1 byte
const unsigned int addr_LocalDay = 0x325;                     //byte 1 byte
const unsigned int addr_LocalMonth = 0x326;                   //byte 1 byte
const unsigned int addr_LocalYear = 0x327;                    //byte 1 byte
const unsigned int addr_EndCoordinateData = 0x327;

const unsigned int addr_TRlocationCRC = 0x340;                //the 16 bit CRC of the last tracker location data is saved here
const unsigned int addr_LocallocationCRC = 0x342;             //the 16 bit CRC of the last local location data is saved here

const unsigned int addr_TestLocation_page3 = 0x3FF;           //used as a location for read\write tests
const unsigned int addr_EndMemory = 0x3FF;
/*********************************************************************
  END GPS CoordinateData
**********************************************************************/


/*
******************************************************************************************************
  mode definitions - used to define which LoRa mode to use
******************************************************************************************************
*/

#define TrackerMode 1
#define SearchMode 2
#define CommandMode 3
#define BindMode 4

#define Portable_Mode 1                             //used as flag and return values
#define Terminal_Mode 2                             //used as flag and return values


