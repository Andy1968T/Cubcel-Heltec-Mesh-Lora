/* Heltec Automation MyNodeMesh sensors
 *
 * Function:
 * 1. Mesh 18 bytes with iD wake at set times to comunicate and then sleep
 * 
 * Description:
 * 1. Only hardware layer communicate, no LoRaWAN protocol support;
 * 2.
 * 3. This example is an adaptive network based on reseting a new sesion after 40 seconds of no comunication
 * Structure is the node places it's self in the network based on Hop count from master
 * and places it's self next one down from an upstream node.
 * Action command codes:
 * 
 * 0xFF only devices with valid data (devices attached send data).Else just be a repeater only
 * 0xFC all devices send sensor data                                      
 * 0xFB matched node send your data. All other nodes just relay it        .... addes not tested
 * Master is esp32 TTGO Lora  connected to Thingspeak server that requests data every 5 minuutes 
 * 
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/ASR650x-Arduino
 *  Hardware Connections:  SHT31-D Humidity & Temp Sensor
* CubeCell Pin  SHT3x Board        Function
* Vext          VIN                Power
* GND           GND                Ground
* SDA           SDA                I2C Data
* SCL           SCL                I2C Clock
  
 ****************************************************/

//#include "LoRaWan_APP.h"
#include "LoRa_APP.h"
//#include"CubeCell_NeoPixel.h"
//#include "Arduino.h"
//#include "Wire.h"
#include <TimeLib.h>
#include <string.h>
#include "SHT3x.h"      // for humidity and temperature sensor
#include "HDC1080.h"    // for humidity and temperature sensor
#include <BH1750.h>    //device for light sensor
#include <BMP180.h>    //device for pressure sensor
#include <BMP280.h>     //device for pressure sensor
#include "HDC1080.h"    //device for humidity and temperature sensor
//#include <MPU9250.h>   //device for motion sensor
#include <ccs811.h>     //device for air quality sensor
#include "Adafruit_PM25AQI.h" //device for air quality sensor
#include <SPL07-003.h>
#include <Bme280.h>     //device for humidity, pressure and temperature sensor
#include <AHTxx.h>     //device for humidity and temperature sensor
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type

Bme280TwoWire BME280;
BMP280 bmp280; // I2C
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
BH1750 lightMeter;
BMP085 bmp180;
HDC1080 hdc1080;
SPL07_003 spa;
//MPU9250 mySensor;
CCS811 ccs;

byte Device;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
uint16_t x_data,y_data,z_data;

//********** */ Uncomment LCD type here
//#define SH107;          //LCD type SH07 (CubeCell Dev-Board Plus (HTCC-AB02)) https://heltec.org/project/htcc-ab02/
//#define SSD1306         // LCD typr SSD1306 (CubeCell GPS-6502 (HTCC-AB02S))  https://heltec.org/project/htcc-ab02s/

//************  Uncoment GPS type here//
//#define GPS_air530      // for heltec with built in GPS ** if you enabe this then you have to move files into path
                        // C:\Users\andre\.platformio\packages\framework-arduinocubecell\libraries\variants\GPS and DISPALY into main path
//#define GPS_Serial1   // Heltec board with second Uart (CubeCell Dev-Board Plus (HTCC-AB02)) https://heltec.org/project/htcc-ab02/
//#define GPS_I2C      // Serial via Gravity Dfrobot i2c to serial module https://www.dfrobot.com/product-2001.html
                        
#ifdef SH107 
#include "HT_SH1107Wire.h"
//extern SH1107Wire  MyDisplay; 
 SH1107Wire  MyDisplay(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , resolution , rst
#define hasLCD
 #endif

#ifdef SSD1306
#include "HT_SSD1306Wire.h"//
SSD1306Wire  MyDisplay(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr , freq , SDA, SCL, resolution , rst
#define hasLCD
#endif

#ifdef GPS_air530
#include "GPS_Air530.h"      // for Heltec cubcell with built in GPS
  //#include "GPS_Air530Z.h"     // for Heltec cubcell built in GPS
  //Air530ZClass GPS;
  Air530Class GPS;
    #define hasGPS
#endif

#ifdef GPS_I2C
#include"CubeCell_TinyGPS++.h"
#include <DFRobot_IICSerial.h>
DFRobot_IICSerial iicSerial1(Wire, /*subUartChannel =*/SUBUART_CHANNEL_1,/*IA1 = */1,/*IA0 = */1);//Construct UART1
TinyGPSPlus GPS;
#define hasGPS
#endif

#ifdef GPS_Serial1
#include"CubeCell_TinyGPS++.h"
TinyGPSPlus GPS;
#define hasGPS
#endif

#ifdef __asr650x__
#include "innerWdt.h"
#endif


// loss of time sync try 10ms seconds back and then try 10ms forward and then add 10ms either way depending battery strength give deviastion as acrussy
// if fail and battery fail imanent then sleep until battery has recoverd to maximise attempts. test voltage at wake up and sleep and display as debug

/*
 * For asr650x, the max feed time is 2.8 seconds.
 * For asr6601, the max feed time is 24 seconds.
 */

//#define MAX_FEEDTIME 2800// ms // was 2800

#define MAX_FEEDTIME 24000// ms


//SHT3x Sensor;


#ifndef LoraWan_RGB 
#define LoraWan_RGB = 0    //set both to 1 for led to work 
#endif
#define RF_FREQUENCY                                868000000 // Hz

#define  TX_OUTPUT_POWER                           20      // dBm // WAS 5
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define  LORA_SPREADING_FACTOR                      12        // [SF7..SF12] was 7
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            900
#define BUFFER_SIZE                                 255 // Define the payload size here
uint32_t SleepMillis ;         // how long was sleep time in millis
uint32_t TxStartTime;
uint32_t TxLength=3000;
//uint32_t RXinterval ;
uint32_t sleepTime;
uint32_t GoSleep;
byte txpacket[BUFFER_SIZE];
byte rxpacket[BUFFER_SIZE];
bool AllTXdone =true;
bool requestDone=false;
int ChanSlot=0;
bool ChanBusy=true;
byte packetList[255];                            // store packets
byte pointer;
float field1=0;
float field2=0;
float field3=0;
byte TXsize=0;
uint64_t TXchipID=getID();
byte HopsMax=7;  // m
byte HopUpID =127;    // HopUpID to move trafic up and down 
byte HopDownID=127;
byte HopID;
byte Command=0x00;  // master request
bool Newsession=true; 
bool blink;
bool VEXstateON;
bool expand=false;  // flip flop expand for sensors that have more than 2 output values
bool ValidData ; // flag set when a sensor has valid data in Fields varables
//float LastLat=51.57861,LastLng=-1.74455;
//float heading;
//uint64_t CompassUpdate;
uint16_t volts;
uint16_t LastVolts;
uint16_t LowVoltage=3650;    // normally 3600
static TimerEvent_t wakeUp;
static TimerEvent_t DeepSleep;
static TimerEvent_t sleep;
float SNR;
int RSSI,lowestRSSI=99;
float LastLat=51.57861,LastLng=-1.74455;
float heading;
uint64_t CompassUpdate;
bool OLEDwash;    // do a screen clear from last display 
bool debug=true;     ///**** debug with print statements
bool debugLED=true;
bool debugGPS=false;
int SleepCount=0;
int reboot=0;
int fracPart(double val, int n)
{
  return (int)((val - (int)(val))*pow(10,n));
}
RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
TimerSysTime_t sysTimeCurrent;
extern volatile unsigned long timer0_millis;
typedef enum
{   
    SLEEPWAIT,
    LOWPOWER,
    TX_SLEEP_WAIT,
    WAIT_TX_DONE,
    LOWBATTERY,
    RX,
    TX
}
States_t;
int16_t txNumber;
States_t state;
int16_t Rssi,rxSize;
// The TinyGPSPlus object
 PM25_AQI_Data aqidata;

void setup() {
    boardInitMcu( );
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    Radio.SetSyncWord(0xF2); 
    Radio.RxBoosted(0);   // boost RX continus
  
   int16_t setRxBoostedGainMode(bool rxbgm, bool persist = true);   // turn on RX gain 
   //RtcInit;
  if (debug==true||debugGPS==true){
    Serial.begin(115200);
  }

  if (debug==true){
      Serial.println("Booting from SetUp");
      Serial.println("------------------------------------------");
    }  
  randomSeed(analogRead(GPIO0)); // Create a random seed from floating GPIO pin
  ChanSlot=random(1,15);
  SetAwakeTime(1000);  // set sleep timer as awaken from lora interupt or fresh start.
  NewSession();

}



void loop() { 

    feedInnerWdt();    // feed watch dog timer even when waiting to sleep
    if (millis()>1000*60*60*24){  // reboot every 24 hours to clear memory leaks
      HW_Reset(0);
    }

      if((state!=LOWPOWER)&&(state!=LOWBATTERY)){  // if not already in low power mode then check battery and go to sleep if low
        if  (LowBatTest()==true){
          if (debug==true){
          Serial.println("");
          Serial.println("LOw Battery quickly send barttery level and sleep");
        
          }
          packetList[0]=0;   //set this pointer to zero to clear transmit buffer list so it only sends one low battery message to network and then sleep until battery recovers
          if (HopID==0){  // if hopID is zero. Nothing has been heard so far 
            HopID=120;  // set hopID far away from Base of zero so has best chance of being repeated to network and not lost as a new node with zero hopID.
          }
          BuildDataPacket();  // find sensor and build the packet
          packetApend();      // append it to the send que
          TXpacketGet();
          TXPacket();         // quickly send it before going to sleep to report low battery to network
         while (state==WAIT_TX_DONE){  // wait for TX to complete before going to sleep
          MyDelay(1000);  // use normal delay so as not to update WDT timer
          }
            Radio.Sleep();   // turn off Lora radio so no interupts from sleep
            MyDelay(4000);  // delay to allow radio to sleep
            SetAwakeTime(100); // Set awake time to sleep the radio
            state=LOWBATTERY;  // Do nothing and wait for sleep timer to goto sleep after 4 seconds 
          }
      }

    DoStuff();
    feedInnerWdt();    // feed watch dog timer
    Radio.IrqProcess( ); 

 
  

  switch(state)
	{
    case TX:
        TXPacket();
        break;

		case RX:
      Radio.Rx( 0 );
      //MyDelay(500);
      if (AllTXdone==true){ // This has woken into a new session and not just to do a TX
        TXpacketGet();
        SetAwakeTime(100);
        state=SLEEPWAIT;  // Go to sleep waiting for session  end
        }
       
        
			break;
  
    case WAIT_TX_DONE:
    
      break; 

		case LOWPOWER:
    
      if (debugLED==true){     
        if ((millis())>(GoSleep+50)){
           digitalWrite(Vext, LOW);    // Turn on Vext for RGB
          turnOnRGB(0x111111,0); //white
          if (millis()>GoSleep+100){
            turnOnRGB(0x000000,0); //off
            VEXToff();  // Turn off vext
            GoSleep=millis();
          }
        }
    }
      lowPowerHandler();

      break;

    default:
        break;
	}
}

void  SendGPS()
  {
    if (Device==0xA1){  // has valid GPS ready to send
      if (HopID==0){  // if hopID is zero. Nothing has been heard
        HopID=120;  // set hopID far away from Base of zero
      Serial.println("Sending GPS Location"); 
      BuildDataPacket();
      packetApend();      // append it to the send que
      AllTXdone=false;
        SetAwakeTime(47000); //give time awake 10 seconds to send gps
      state=TX;
      }
    }
  }

void DoStuff(){
  char c;
  #ifdef GPS_air530
   while (GPS.available() > 0)  // onboard GPS
    {
      c=(GPS.read());
      if (debugGPS==true){
      Serial.print(c);
      }
      GPS.encode(c);
    }
    #endif

    #ifdef GPS_Serial1
   
    while (Serial1.available() > 0) // external Serial GPS 
   {
  
     c=(Serial1.read());
      if (debugGPS==true){
      Serial.print(c);
     }
      GPS.encode(c);
    }
    #endif

    #ifdef GPS_I2C
      while(iicSerial1.available()){
        c = iicSerial1.read();/*Read data of UART1 receive buffer */
        if ((debugGPS==true)){
          Serial.print(c);
        }
      
        GPS.encode(c);
      }
  #endif  
    
    #ifdef hasGPS
    
 
    if (GPS.location.isUpdated()){
        updateCompass();
    

    if (GPS.location.isValid())
    {
        
          field2 = (float)GPS.location.lng();
          field3 = (float)GPS.location.lat(); 
          if ((field2!=0)&&(field3!=0)){
            Device=0xA1;
            ValidData=true;
          }
        #ifdef hasLCD    // Don't turn off GPS. Keep showing track on display
          return;
        #endif
          #ifdef GPS_Serial1
            Serial1.print("$PGKC105,8*3F\r\n"); // automatic power save mode wake on serial
          #endif

          #ifdef GPS_air530
            GPS.sendcmd("$PGKC105,8*3F\r\n");
          #endif

          #ifdef GPS_I2C
            iicSerial1.print("$PGKC105,8*3F\r\n");
            VEXToff();
          #endif
        }
  }
  #endif
}



void TXPacket(){

        TxNodeIDPrint();    // display the TX packet ID 
        feedInnerWdt();    // feed watch dog timer Next bit of code could get stuck here if radio busy
          uint32_t WaitTimeout=millis()+20000; // 5 second timeout
          ChanBusy=false;
           if (debug==true){
          Serial.print("  Slot:");
          Serial.print (ChanSlot);
          Serial.print("       ");
          TXPacketPrint();
        }
       while (!Radio.IsChannelFree( MODEM_LORA,RF_FREQUENCY , lowestRSSI, LORA_SPREADING_FACTOR*100)&&(millis()<WaitTimeout))  // wait for channel to be free
        {
           SetAwakeTime(5000); // keep awake while channel busy
          ChanSlot=random(1,15);
          ChanBusy=true;
            MyDelay(TxLength); 
        if (debug==true){
                Serial.print("X");
            }
        if (debugLED==true){
            turnOnRGB(0x111100,0); //yellow
          } 
        }

        if (!Radio.IsChannelFree( MODEM_LORA,RF_FREQUENCY , lowestRSSI, LORA_SPREADING_FACTOR*100)){
        lowestRSSI=lowestRSSI+1; // reset lowest because it got stuck as channel busy
        SetAwakeTime(5000); // keep awake while channel busy
        MyDelay(TxLength);
        if (debug==true){
                Serial.print("x");
            }
        }

       if (debugLED==true){
          turnOnRGB(0x110000,0); //red
        }

      SetAwakeTime(10000); // 
      TxStartTime=millis(); //measure the length of a Transmit to calculate slot time length
       Radio.Send( (uint8_t *)txpacket,18); // *** packet length is 18 
      state=WAIT_TX_DONE;
      }


void OnTxDone( void )
{ 
    TxLength=(millis()-TxStartTime)+100;
    VEXToff();  // Turn off vext as Lora leaves it on  
    if (debugLED==true){
          turnOnRGB(0,0);
          VEXToff();  // Turn off vext
          
      }
      Radio.Rx(0);  // go back to RX continus
      state=RX;  // make sure it returns to loop
      AllTXdone=true;
}

void OnTxTimeout( void )
{   
    if (debug==true){
    Serial.println("  ****  TX Timed out *****");
    }
    state=RX;
    AllTXdone=true;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{   
    Newsession=false;  // reset new session flag as we have had comunication so not a new session anymore
    innerWdtEnable(true);   //  set inner watchdog to manual feeding 
    SetAwakeTime(6000);  // set sleep timer as awaken from lora interupt or fresh start. 
    rxSize=size;
    if (debugLED==true){
      turnOnRGB(0x001100,0); //green
      MyDelay(30);
      turnOnRGB(0,0);
      VEXToff(); 
    }
    
    if (rxSize==18){  // found a potential node packet
        SNR=snr;
      
      RSSI=rssi;
      if (RSSI<lowestRSSI){
        lowestRSSI=RSSI;  // store lowest RSSI value for channel test
      }
      memcpy(rxpacket, payload, size );
      if (debug==true){     
        RxNodeIDPrint();
        RXPacketPrint();
        }

      if (((rxpacket[6] & B10000000)==0)&&((rxpacket[6]& B01111111)<HopUpID)){ // downstream packet 
        HopUpID=rxpacket[6] ;  // change this nodes HopUpID to How many node hops from master requester
        
        if (HopUpID<HopsMax){
        HopUpID++;
      }

        HopID=HopUpID;
        packetApend();   // pass request on 
        int cmd=rxpacket[7];

        if ((cmd==0xFC)&&(requestDone==false)) {// all devices send their devis data
          if (ValidData==false){
            SearchSence();     // find sensor data and save it in fields
          }
            BuildDataPacket();  // build the packet
            packetApend();      // append it to the send que
            requestDone=true;
        }
        if ((cmd==0xFF)&&(requestDone==false))  { // only devices with valid data (devices attached send data). Just be a repeater only
          if (ValidData==false){
            SearchSence();  
          }
            if (ValidData==true){
            BuildDataPacket();  // build the packet
            packetApend();      // append it to the send que
            requestDone=true; 
          }
        
      }


        if ((cmd==0xFB)&&(requestDone==false))// only matched device send data
         {
         
          bool matched=true;
          byte aStr[8]={0};
          int64ToChar(aStr, TXchipID);

          for (int i=0;i<6;++i) {
            if ( rxpacket[i]!=aStr[i])
                matched=false;
            }
          if (matched==true)  {
              SearchSence();     // find sensor data and save it in fields
              BuildDataPacket();  // find sensor and build the packet
              packetApend();      // append it to the send que
              requestDone=true;
          }
  
        }  
      }


  if ((rxpacket[6] & B10000000)!=0)   {   // Upstream packet bit set
          
    
           if ((rxpacket[6] & B01111111)>HopDownID)  {    //  this is upstream of this node? 
            HopID= HopDownID; 
            packetApend();  //pass packet down  
            }

             if (HopDownID==127)  {    // Is Packet the first upstream packet
            HopDownID=(rxpacket[6] & B01111111);
            if (HopDownID>1){
              HopDownID--;
            }

            if (HopDownID==0){   // It can't be 0 as thats the base value
              HopDownID=1;
            }
            HopID= HopDownID;
            packetApend();  //pass packet down 
              }


          if ((rxpacket[6] & B01111111)<HopDownID)  {    // Downsteam packet is lower so go lower 
            HopDownID=(rxpacket[6] & B01111111);

             if (HopDownID>1){
              HopDownID--;
            }

            if (HopDownID==0){   // It can't be 0 as thats the base value
              HopDownID=1;
            }
              }
            }
     
      
    }
    if(AllTXdone==false){ // Device has woken while waiting for a TX due recieving another packet
      state=TX_SLEEP_WAIT;  // go back to sleep waiting for TX to complete
    }
    else {
      state=RX;  // go back to RX waiting for next packet
    }
    SetAwakeTime(100);  // set sleep timer to wait for TX to complete before sleeping 
  }



void BuildDataPacket(){   // Build data packet
                      // so build a packet to request time
    
    byte aStr[8]={0};
    byte bStr[8]={0};
    byte cStr[8]={0};

    int64ToChar(aStr, TXchipID);
    for (int i=0;i<6;++i) {
      rxpacket[i]=aStr[i];
    }
    rxpacket[6]=(HopID | B10000000); // This nodes HopUpID and set bit 7 as direction towards master
    if ((ValidData==false)&&(Device<128)){
      Device=0;   // zero as the Data is invalid for a i2c sensor
}
    rxpacket[7]=Device;  // action is sensor type
     if (Device==0xA1){      // this is a GPS device so send SNR and not battery voltage
        field1=float(SNR);  // just add the SNR  long/lat fields already filled
        }
    int16ToChar(bStr, field1); // two bytes of voltage
    for (int i=0;i<2;++i) {
      rxpacket[8+i]=bStr[i];
    }
    
    floatToChar(cStr,field2);  // 4 bytes of field 2 value  filled blank
    for (int i=0;i<4;++i) {
      rxpacket[10+i]=cStr[i];
    }
    floatToChar(cStr,field3); // 4 bytes of field 3 value filled blank
    for (int i=0;i<4;++i) {
      rxpacket[14+i]=cStr[i];
    }
    if (Device==0){    // inject my loction clear text SN34LY-19
      rxpacket[10]=0x53;
      rxpacket[11]=0x4E;
      rxpacket[12]=0x33;
      rxpacket[13]=0x34;
      rxpacket[14]=0x4C;
      rxpacket[15]=0x59;
      rxpacket[16]=0x2D;
      rxpacket[17]=0x31;
      rxpacket[18]=0x39;


    }
    
  }

void TXpacketGet(){
  
  byte BasePoint=0;
  pointer=0;
  TXsize=0;
  while (packetList[BasePoint+pointer]!=0)  {
    
    TXsize=packetList[BasePoint+pointer]-2;
      pointer++;
      if (packetList[BasePoint+pointer]<2){
        packetList[BasePoint+pointer]++;  // increment it as going to transmit this one
        
        pointer++; // increment to point at packet data
        for (int i=0;i<TXsize;++i) {
          txpacket[i]=packetList[BasePoint+pointer];
          pointer++;
        }
        AllTXdone=false;
        return;
      }
    
    BasePoint+=20;
    pointer=0;
  }

  TXsize=0;
 AllTXdone=true;
}

void packetApend(){
  rxpacket[6]=(rxpacket[6]&B10000000);   // preserve the direction bit
  rxpacket[6]=(rxpacket[6]|HopID);   // merge in this nodes HOP id
  byte BasePoint=0;
  pointer=0;
  while (packetList[BasePoint+pointer]!=0){  // look for end zero
  
  bool match=true;
  pointer++; // Now pointing at TX RX count
  pointer++; // Now pointing at Data
  for (int i=0;i<18;++i) {
    if ((packetList[BasePoint+pointer]!=rxpacket[i])&&(i!=6)){  // Compare the packet data except for byte [6] which is the changing hop count
      match=false;
    }
    pointer++;
  }
   if (match==true) {                    // packet already in list don't append just add recieved count and return

    return;
   }
  
  //BasePoint+=packetList[BasePoint]; // point to start of next packet and test match again
  BasePoint+=20;
  pointer=0;
  }
  if (BasePoint+18>254){
    if (debug==true){
    Serial.print("");
    Serial.println("Buffer is full");
    }
    return;        // if buffer full do not append
  }
  packetList[BasePoint+pointer]= 20;
  pointer++;
  packetList[BasePoint+pointer]=1; // retransmit recieved packet once
  pointer++;

  for (int i=0;i<18;++i) {// add payload to list
    packetList[BasePoint+pointer]=rxpacket[i];
    pointer++;
  }
  packetList[BasePoint+pointer]=0;   // End terminated ready for appending
    if (state!=TX_SLEEP_WAIT){// TX already pending after sleep so don't change state if already pending
      state=RX;
    }
}

void RxNodeIDPrint(){
  
    byte aStr [8]={0};
      
    for (int i=0;i<6;++i) {   // get RX Chip ID
      aStr[i]=rxpacket[i];
    }
    int64_t RXchipID = charTo64bitNum(aStr);
    if (debug==true){
      Serial.println("");
    if (rxpacket[6] & B10000000){
      Serial.print(" v");
    }
      else 
      {
      Serial.print(" ^");  
      }
    
    Serial.print(" HOP ");
    Serial.print(rxpacket[6]& B01111111,HEX);
    Serial.print(" Action ");
    Serial.print (rxpacket[7],HEX);
    
    Serial.printf(" RX-ChipID:%04X%08X",(uint32_t)(RXchipID>>32),(uint32_t)RXchipID);
    Serial.print(" > ");
    Serial.print("SNR:");
    Serial.print(SNR);
    Serial.print (" RSSI:");
    Serial.print (RSSI);
  }
 #ifdef hasLCD
    char str[30];
    MyDisplay.clear();
    // int angle=180;
    MyDisplay.setColor(WHITE);
    MyDisplay.setFont(ArialMT_Plain_16);
    int index = sprintf(str,"%04X%08X\r\n",(uint32_t)(RXchipID>>32),(uint32_t)RXchipID);
    str[index] = 0;
    MyDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
    MyDisplay.drawString(0, 0, str);

    if (rxpacket[6] & B10000000){
      index=sprintf(str,"v");
    }
      else 
      {
     index=sprintf(str,"^");  
      }
      str[index] = 0;
    MyDisplay.drawString(40, 20, str);

    index = sprintf(str,"HOP %01x\r\n",(rxpacket[6]& B01111111));
    str[index] = 0;
    MyDisplay.drawString(0, 40, str);

    MyDisplay.setFont(ArialMT_Plain_24);

    index = sprintf(str,"RX");
    str[index] = 0;
    MyDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
    MyDisplay.drawString(0, 16, str);
    MyDisplay.display();
  #endif
  
  
}

void TxNodeIDPrint(){
    byte aStr [8]={0};
   
    for (int i=0;i<6;++i) {   // get  TX Chip ID
      aStr[i]=txpacket[i];
    }
    
    int64_t TXchipID = charTo64bitNum(aStr);
    if (debug==true){
      Serial.println("");
    if (txpacket[6] & B10000000){
      Serial.print(" v");
    }
      else 
      {
      Serial.print(" ^");  
      }
    Serial.print(" HOP ");
    Serial.print(txpacket[6]& B01111111,HEX);
    Serial.print(" Action ");
    Serial.print((txpacket[7]),HEX);
    Serial.printf(" TX-ChipID:%04X%08X",(uint32_t)(TXchipID>>32),(uint32_t)TXchipID);
    Serial.print(" > ");
    
    #ifdef hasLCD
    char str[30];
    MyDisplay.clear();
    MyDisplay.setColor(WHITE);
    MyDisplay.setFont(ArialMT_Plain_16);
    int index = sprintf(str,"%04X%08X\r\n",(uint32_t)(TXchipID>>32),(uint32_t)TXchipID);
    str[index] = 0;
    MyDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
    MyDisplay.drawString(0, 0, str);
    if (txpacket[6] & B10000000){
      index=sprintf(str,"v");
    }
      else 
      {
     index=sprintf(str,"^");  
      }
      str[index] = 0;
    MyDisplay.drawString(40, 20, str);
  
    index = sprintf(str,"HOP %01x\r\n",(txpacket[6]& B01111111));
    str[index] = 0;
    MyDisplay.drawString(0, 40, str);
    
    MyDisplay.setFont(ArialMT_Plain_24);
    index = sprintf(str,"TX");
    str[index] = 0;
    MyDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
    MyDisplay.drawString(0, 16, str);
    MyDisplay.display();
  #endif
  }
  
  

}
void DisplayGPSInof()
{
  #ifdef hasLCD
  char str[30];
  MyDisplay.clear();
  MyDisplay.setColor(WHITE);
  MyDisplay.setFont(ArialMT_Plain_10);
  int index = sprintf(str,"%02d-%02d-%02d",GPS.date.year(),GPS.date.day(),GPS.date.month());
  str[index] = 0;
  MyDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  MyDisplay.drawString(0,0,str);
  
  index = sprintf(str,"%02d:%02d:%02d",GPS.time.hour(),GPS.time.minute(),GPS.time.second(),GPS.time.centisecond());
  str[index] = 0;
  MyDisplay.drawString(60,0,str);

  if( GPS.location.age() < 1000 )
  {
   
    MyDisplay.drawString(120,0, "A");
  }
  else
  {
   
    MyDisplay.drawString(120,0,"V");
  }
  
  index = sprintf(str,"alt: %d.%d",(int)GPS.altitude.meters(),fracPart(GPS.altitude.meters(),2));
  str[index] = 0;
  MyDisplay.drawString(0,16,str);
   
  index = sprintf(str,"hdop: %d.%d",(int)GPS.hdop.hdop(),fracPart(GPS.hdop.hdop(),2));
  str[index] = 0;
  MyDisplay.drawString(0,32,str); 
 
  index = sprintf(str,"lat :  %d.%d",(int)GPS.location.lat(),fracPart(GPS.location.lat(),4));
  str[index] = 0;
  MyDisplay.drawString(60,16,str);   
  
  index = sprintf(str,"lon:%d.%d",(int)GPS.location.lng(),fracPart(GPS.location.lng(),4));
  str[index] = 0;
  MyDisplay.drawString(60,32,str);

  index = sprintf(str,"speed: %d.%d km/h",(int)GPS.speed.kmph(),fracPart(GPS.speed.kmph(),3));
  str[index] = 0;
  MyDisplay.drawString(0, 48, str);
  MyDisplay.display();
  OLEDwash=true;
  #endif
}

void RXPacketPrint(){
  byte aStr [8]={0};
  byte bStr [8]={0};
  byte cStr [8]={0};
  int pointer =0;
    
  for (int i=0;i<6;++i) {   // get Chip ID
    //aStr[i]=rxpacket[pointer];
    pointer++;
  }
  pointer++;
  pointer++;
  for (int i=0;i<2;++i) {   // get field 1 as char
      aStr[i]=rxpacket[pointer];
      pointer++;
    }
    
    for (int i=0;i<4;++i) {      // get field2 as char
      bStr[i]=rxpacket[pointer];
      pointer++;
    } 
    

     for (int i=0;i<4;++i) {    // get field3 as char
      cStr[i]=rxpacket[pointer];
      pointer++;
    }
    
     DataPrint(rxpacket[7],charTo16bitNum(aStr),charTofloatNum(bStr),charTofloatNum(cStr));
    
}

void TXPacketPrint(){
  byte aStr [8]={0};
  byte bStr [8]={0};
  byte cStr [8]={0};
  int pointer =0;
    
  for (int i=0;i<6;++i) {   // get Chip ID
   // aStr[i]=txpacket[pointer];
    pointer++;
  }
  pointer++;
  pointer++;
  for (int i=0;i<2;++i) {   // get battery  or SNR voltage from packet
      aStr[i]=txpacket[pointer];
      pointer++;
    }
    
    for (int i=0;i<4;++i) {      // get field1 as a float
      bStr[i]=txpacket[pointer];
      pointer++;
    } 

     for (int i=0;i<4;++i) {    // get field2 as a float
      cStr[i]=txpacket[pointer];
      pointer++;
    }
        DataPrint(txpacket[7],charTo16bitNum(aStr),charTofloatNum(bStr),charTofloatNum(cStr));
      
     

}

void DataPrint(int device, float field1,float field2,float field3){  
  switch(device)  // packet location of sensor type
  {

  case 0x00:{     // No device found
    Serial.print(" Battery: ");
    Serial.print((float)field1/1000); 
    Serial.print("V No i2C found");
    break;
  }
  case 0x01: {// reply code for volts temperature humidity 
      Serial.print(" Battery:");
      Serial.println((float)field1/1000);
      Serial.print(" T=");
      Serial.print(field2);
      Serial.write("\xC2\xB0"); //The Degree symbol 
      Serial.print("C");
      Serial.print(" Humidity=");
      Serial.print(field3);
      Serial.print("%");

      break;
    }
    case 0x02: {// reply code for there is no sensor or broken 
    
    
      Serial.print("Cell has no sensor");
  
      break;
    }
    case 0x12: // partical sensor
    {
      Serial.print(" Battery:");
      Serial.print((float)field1/1000);
      Serial.print(" Partical Sensor ");
      Serial.print(" PM2.5=");
      Serial.print(field2);
      
      Serial.print(" PM1.0=");
      Serial.print(field3);
      break;
    }

  case 35: //0x23 -- BH1750 light sensor
    { 
      Serial.print(" Battery:");
      Serial.print((float)field1/1000);
      Serial.print(" :0x23 -- BH1750 light sensor");
      Serial.print(" :Light=");
      Serial.print(field2);
      Serial.print(" lx");
      break;
    }
   case 64: //0x40 -- HDC1080 temperature and humidity sensor
    {
      Serial.print(" Battery:");
      Serial.print((float)field1/1000);
      Serial.println(" :HDC1080 Sensor");
      Serial.print(" T=");
      Serial.print(field2);
      Serial.write("\xC2\xB0"); //The Degree symbol
      Serial.print("C ");
      Serial.print(" RH=");
      Serial.print(field3);
      Serial.print("%");
      }
      break; 
  case 0x38: //0x38 -- AHT10 temperature and humidity sensor
    {
      Serial.print(" Battery:");
      Serial.print((float)field1/1000);
      Serial.print(" :AHT10 Sensor");
      Serial.print(" T=");
      Serial.print(field2);
       Serial.write("\xC2\xB0"); //The Degree symbol
      Serial.print("C");
      Serial.print(" RH=");
      Serial.print(field3);
      Serial.print("%");
      }
      break; 
  

  case 90: {//0x5a -- CCS811 Air quality sensor
      Serial.print(" Battery:");
      Serial.print((float)field1/1000);
      Serial.print(" CCS811 Air quality sensor ");
      Serial.print(" CO2=");
      Serial.print(field2);
      Serial.print(" ppm");
      Serial.print(" TVOC=");
      Serial.print(field3);
      Serial.print("ppb");
      break;  
  }

  case 0x75: {// reply code for volts temperature humidity 
    
      Serial.print(" Battery:");
      Serial.print((float)field1/1000);
  
      Serial.print(" RH=");
      Serial.print(field2);
      Serial.print("% "); //The Degree symbol
      Serial.print(" Pressure=");
      Serial.print(field3/100);
      Serial.print("hPa");
      break;
  }
  case 0x76: {// reply device address for volts temperature humidity 
       Serial.print(" Battery:");
      Serial.print((float)field1/1000); 
    
      Serial.print(" T=");
      Serial.print(field2);
      Serial.write("\xC2\xB0"); //The Degree symbol
      Serial.print("C ");
      Serial.print("Pressure=");
      Serial.print(field3/100);
      Serial.print("hPa");
      break;
  }
  case 0x77: {// reply device address for volts temperature humidity 
       Serial.print(" Battery:");
      Serial.print((float)field1/1000); 
    
      Serial.print(" SPA06 Sensor T=");
      Serial.print(field2);
      Serial.write("\xC2\xB0"); //The Degree symbol
      Serial.print("C ");
      Serial.print("Pressure=");
      Serial.print(field3/100);
      Serial.print("hPa");
      break;
  }
  case 0x3C: {//GPS with small screen
    Serial.print(" GPS module with screen");
    Serial.print(" Battery: ");
    Serial.print((float)field1/1000);
    Serial.print(" No GPS Fix ");
    break;
  case 0xA1: {//GPS location
      #ifdef  hasLCD
        DisplayGPSInof();
      #endif
      Serial.print(" GPS module:");
      Serial.print(" SNR: ");
      Serial.print((float)field1);
      Serial.print(" Longatude: ");
      Serial.print(field2);
      Serial.print(" Latitude:");
      Serial.print(field3);
      break;
  }
  case 0x70: {//GPS location with battery
      Serial.print(" GPS module:");
      Serial.print(" Battery: ");
      Serial.print((float)field1/1000);
      Serial.print(" Longatude: ");
      Serial.print(field2);
      Serial.print(" Latitude:");
      Serial.print(field3);
      break;
  }
  // master commands *************************************************************
  case 0xFC:{
      Serial.print(" Master request all devices respond ") ;
      
      break;
  }
  case 0xFF:{
    Serial.print(" Master request only devices with valid data respond") ;
    
    break;
}
 //               ******************************************************************
  case 0x56:{ //Unidentified device
      Serial.print(" Battery:");
      Serial.print((float)field1/1000); 
      Serial.print(" Unidentified Device") ;
      break;
  }
  case 0x7F:{ //Unidentified device
      Serial.print(" Battery:");
      Serial.print((float)field1/1000); 
      Serial.print(" No I2c device connected");
      break;
  }
  Serial.print(" No Device!");
  }
}
}


void onSleep()
{  

  if (debugLED==true){
          turnOnRGB(0x000011,0); //flash led blue before sleep
          MyDelay(30);
          turnOnRGB(0,0);
          VEXToff();  // Turn off vext
          MyDelay(30);
        }
  #ifdef GPS_Serial1
  #ifndef hasLCD
    Serial1.print("$PGKC105,8*3F\r\n"); // automatic power save mode wake on serial
    MyDelay(500);
    Serial1.flush();
    Serial1.end();
   #endif
 #endif

#ifdef GPS_air530
  #ifndef hasLCD   // don't turn of GPS
     GPS.sendcmd("$PGKC105,8*3F\r\n");
  #endif
#endif

#ifdef GPS_I2C
  iicSerial1.print("$PGKC105,8*3F\r\n");
  VEXToff();
#endif
 
       #ifdef hasLCD
          OLEDsleep();
          sleepTime=1; // if there is LCD then just run it flat 
      #endif
      if(state==LOWBATTERY){  // if we are already in low power mode then we must have woken to do a TX but battery is low so go back to sleep for a long time
        sleepTime=1000*60*60*6; // sleep for 6 hours if battery low and woken for a TX as there is no point doing the TX if battery is low
        if (debug==true){
          Serial.println("");
          Serial.print(" Low Battery - Going to Sleep ");
          Serial.print(sleepTime/(1000*60*60));
          Serial.println(" Hours ");
        }
      } 
      else {

       if (AllTXdone==false){  // if there is still a TX to do so sleep for chan slot time  
       sleepTime=ChanSlot*TxLength; // sleep for the remainder of the slot time     
      }
      else 
      {
        sleepTime=33000; // sleep  wait for next session
         } 
        }

      if (debug==true){
        //Serial.println("");
       // Serial.print(" Sleep for: ");
        //Serial.print (sleepTime/1000);
       // Serial.print(" seconds ");
       // Serial.print("VOLTS=");
       // Serial.print((float)volts/1000);
        delay(100); // just to make sure all the above prints out before we turn off serial
      }
        Serial.flush();
       // Serial.end();
        Wire.end();
      pinMode(GPIO4, INPUT_PULLDOWN);  
      digitalWrite(Vext,HIGH); // turn off power to the sensor
      innerWdtEnable(false);   //  set inner watchdog to off before sleep
      wdt_isr_Stop();     // disable the wDT interrupt before sleep
      
      CySysWdtDisable();
      CySysPmDeepSleep;   // use to turn off sys wdt
        // put this here because V1 carry on and don't actulally run from onwake
      TimerInit( &wakeUp, onWake );
      TimerSetValue( &wakeUp, sleepTime);
      TimerStart( &wakeUp );
      VEXToff();   
      state=LOWPOWER;
      GoSleep=millis();
}

void SetAwakeTime(uint32_t sleepTime){
      TimerInit( &sleep, onSleep );
      TimerSetValue( &sleep, sleepTime); 
      TimerStart( &sleep );
}

void onWake(){
TimerStop(&sleep);
TimerStop(&wakeUp);
VEXToff();
innerWdtEnable(true);   //  set inner watchdog to manual feeding
sleepTime=1000*60*60;  // New seesion started so go to sleep for an hour until woke from RX interrupts
SetAwakeTime(33000);  // set sleep timer as awaken from lora interupt or fresh start. 
if (AllTXdone==true){
  state=RX;
}
else {
  state=TX;
}

Radio.Rx(0);

  if (debug==true||debugGPS==true){
    Serial.begin(115200);
  }

  if (debugLED==true){
          turnOnRGB(0x000011,0); //flash led blue on wake
          MyDelay(30);
           turnOnRGB(0x001100,0); //flash led green on wake
          MyDelay(30);
          turnOnRGB(0x110000,0); //flash led red on wake
          MyDelay(30); 
          turnOnRGB(0,0);
          VEXToff();  // Turn off vext
        }
if ((AllTXdone==true)){ // This has woken into a new session and not just to do a TX
  NewSession(); 
}

}

bool LowBatTest(){
  volts = getBatteryVoltage();
  #ifdef hasLCD
   return false;  // battery always ok when on lcd
   #endif
   
  if ((volts)<LowVoltage){ // battery going flat so sleep 3850
  return true;
  } 
  return false;
}
  
void  NewSession(){
  Newsession=true;
  requestDone=false;
  reboot++;
  if ((reboot>50)){
    HW_Reset(0);     //do a reboot to clear out any crap and  make a fresh start
  }
  if (debug==true){
    Serial.println("");
    Serial.print("----New Session---- ");
    Serial.print(reboot);
    TXchipID=getID(); 
    Serial.printf(" ChipID:%04X%08X",(uint32_t)(TXchipID>>32),(uint32_t)TXchipID); 
  }
  randomSeed(analogRead(GPIO0)); // Create a random seed from floating GPIO pin
  volts=getBatteryVoltage();
  field1=volts;
  field2=0;
  field3=0;
  SleepCount=0;
  pinMode(GPIO4, OUTPUT);
  //pinMode(GPIO6, OUTPUT);
  pinMode(Vext, OUTPUT);
  //digitalWrite(GPIO7,HIGH); // turn on 3.3V for LoRa
  //VEXToff();   // it wakes up on so turn off until needed

    #ifdef GPS_I2C    // run this if GPS is fitted to start GPS on boot
      VEXTon();
      I2C_Stop(); 

      aqi.begin_I2C();
      iicSerial1.begin(/*baud = */9600);/*UART1 init*/
      Wire.setClock(120000);
      iicSerial1.println("$PGKC161,2,500,1000*03\r\n"); // wakes up GPS  by setting PPS flash rate 3D fix
    #endif

    #ifdef GPS_air530
      Serial.begin(115200);  //uses serial to let us know it's auto baud rate 
      GPS.end();
      GPS.begin();
      MyDelay(500);
      GPS.setmode(MODE_GPS_GALILEO_GLONASS); 
      GPS.setNMEA (NMEA_GGA|NMEA_GSA|NMEA_RMC|NMEA_VTG);
      GPS.sendcmd("$PGKC161,2,500,1000*03\r\n"); // wakes up GPS  by setting PPS flash rate 3D fix
  #endif 

  #ifdef GPS_Serial1
    Serial1.begin(115200);
    Serial1.printf("$PGKC105,8*3F\r\n"); // automatic power save mode wake on serial
  #endif
  ValidData=false;
// new session so clear all variables for hop management
requestDone=false;
HopUpID=127;   // set this to highest it can be for this session
HopDownID=127; 
HopID=0;
packetList[0]=0; //set this pointer to zero to clear transmit buffer list 
Radio.Rx(0);  // set Lora to receive mode
      #ifdef hasLCD  //keep listening for 1hr as has LCD
      SetawakeTime(1000*60)
    #endif   

  #ifdef hasLCD
   OLEDwake();
   #endif
   }

void SHT30() {
  //Sensor.UpdateData();

  if (field3==0){        // error reading  humidity so try again
   //Sensor.UpdateData(); 
  }
if (field3==0){        // error reading third time lucky
  // Sensor.UpdateData(); 
  }

  //field2=Sensor.GetTemperature();
  //field3=Sensor.GetRelHumidity(); 

     //Wire.end();
     MyDelay(50);
    //digitalWrite(GPIO0,HIGH);//set HiIGH GPIO0 to disable CCS811
}
void SearchSence(){
  SetAwakeTime(40000); // keep device awake while searching for i2c devices and reading them
expand=!expand; // flip expand mode for BME280 for reading temp and pressure or humidity and pressure
VEXTon();   // time in syn so turn on vext ready to read I2C devices as normal 
digitalWrite(GPIO4,HIGH);
  MyDelay(5000); // give time for partical sensor to spin up and report correct address
  if (ValidData==false){
  I2C_Scan();

  if (debug==true){
    Serial.println("");
    Serial.print("Device Found at Address: 0x");
    Serial.print(Device,HEX);
    }
  
  Device;   // set the action for the type of device found
  }

  switch(Device)
  {
    

    case 0xA1: // GPS onboard
    {
    
        if (ValidData==true){
        Serial.print("SNR=");
        Serial.println(SNR);
        field1=float(SNR);  // just add the SNR  long/lat fields already filled
        }
      break;
    }
    
    case 0x12: //0x12 -- Partical sensor
    {
      SetAwakeTime(47000); // keep device awake while while waiting to suck in air and get a reading
      MyDelay(6000);   // suck in lots of air
      if(aqi.read(&aqidata)) {
        field1=volts;
        field2 = aqidata.pm25_standard;
        field3 = aqidata.pm100_standard;
        ValidData=true;
      }
        VEXToff(); // all done, turn off power
      break;
    }
    case 0x23: //-- BH1750 light sensor
    {
      field2 = lightMeter.readLightLevel();
      ValidData=true;
      VEXToff(); // all done, turn off power
      break;
    }

    case 0x38: //0x38 -- AHT10
    {
      aht10.begin();
      MyDelay(500);
      field2 = aht10.readTemperature(); //read 6-bytes via I2C, takes 80 milliseconds
      field3 = aht10.readHumidity(); //read another 6-bytes via I2C, takes 80 milliseconds

      ValidData=true;
      VEXToff();
      //digitalWrite(Vext,HIGH); // all done, turn off power
      break;
    }
  
    case 0x76: //-- BME280 Barometer temperature Humidity Fake Module should be 0x77
    {         // Needed to swap 0x77 with 0x76 in adafruitBME280.h to get to work
    
      BME280.begin(Bme280TwoWireAddress::Primary);
      BME280.setSettings(Bme280Settings::indoor());
      MyDelay(100);
      if (expand==true){
      field2 =BME280.getTemperature() ;
  

      }
      else{
       Device=0x75;  // change the device code as if it was another device that just reads Humidity and pressure
        field2 =BME280.getHumidity() ;
      }
      field3 =BME280.getPressure() ; 
      ValidData=true;
      if(isnan(field3)){  // check for failed result in pressure field3
        ValidData=false;
        if (debug==true){
          Serial.println("Failed to read correctly BM280");
        }
      }
      VEXToff(); // all done, turn off power
      break;
    }

    case 0x77: //0x77 -- BMP280 Barometer sensor
    {
     spa.begin();
      spa.setPressureConfig(SPL07_4HZ, SPL07_1SAMPLE);
    spa.setTemperatureConfig(SPL07_4HZ, SPL07_1SAMPLE);
  MyDelay(100);

  // Set SPL07-003 to continuous measurements
  spa.setMode(SPL07_ONE_PRESSURE);
  spa.setMode(SPL07_ONE_TEMPERATURE);
          field2 = spa.readTemperature();
          field3 = spa.readPressure();
      ValidData=true;
      VEXToff();
      //digitalWrite(Vext,HIGH); // all done, turn off power
      break;
    }

    case 0x40: //-- HDC1080 temperature and humidity sensor
    {
      hdc1080.begin(0x40);
      field2=hdc1080.readTemperature();
      field3=hdc1080.readHumidity();
      ValidData=true;
      VEXToff(); // all done, turn off power
      break;
    }
    
    case 0x5A: //-- CCS811 Air quality sensor
      {
      ccs.begin(0x5A);
      while(!ccs.available()){

      }
            MyDelay(750);
            ccs.readData();
            field2=ccs.geteCO2();
            field3=ccs.getTVOC();
            ValidData=true;
            VEXToff();// all done, turn off power
        break;
      }
     
     
      
  }
   #ifdef hasLCD  // so don't turn off wire
      return;
  #endif
  Wire.end();
	//digitalWrite(GPIO4,LOW);
 // digitalWrite(GPIO0,HIGH);//set GPIO0 to enable CCS811
 VEXToff();// all done, turn off power 
}

void I2C_Scan()
{
  I2C_Stop();   // this resets the wire() http://community.heltec.cn/t/cubecell-vext-with-i2c-sensors/886
  Wire.begin();
  Wire.setClock(10000);
  byte error;
  int nDevices;
  
  for(Device = 1; Device < 127; Device++ ) // normally starts at 1 
  {
    Wire.beginTransmission(Device);

    error = Wire.endTransmission();

    if (error == 0)
    {
      nDevices++;
      Wire.setClock(400000); // speed clock back up for Lora not to timeout
      break;
    }
    
  }
Wire.setClock(400000); // speed clock back up for Lora not to timeout
}
void updateCompass()
{
  int x,y;
  #ifdef hasLCD;
    if (GPS.location.isUpdated()&&GPS.location.isValid()){
    if (OLEDwash==true){   // screen needs a complete clear
      MyDisplay.clear();
      OLEDwash=false;
    }
    MyDisplay.setColor(BLACK);
    MyDisplay.fillRect(32,24,60,16); //delete text
    MyDisplay.fillRect(0,44,92,16); // delete text
    x = int(14 * sin(heading));
    y = int(14 * cos(heading));
    MyDisplay.fillCircle(110, 35, 13);
    char str[30];
    MyDisplay.setFont(ArialMT_Plain_16);
    MyDisplay.setTextAlignment(TEXT_ALIGN_RIGHT);
    MyDisplay.setColor(WHITE);
    int Kmph=int(GPS.speed.kmph());
    if (Kmph>3){

      SetSleepTime(30000);  // set sleep timer
      int index = sprintf(str,"%dKm/h",Kmph);
    
    str[index] = 0;
    MyDisplay.drawString(95,24,str);
  int distance=int(GPS.distanceBetween(LastLat, LastLng,GPS.location.lat(),GPS.location.lng()));
  if (distance<1000){
    index = sprintf(str,"%dMtr",distance);
  }
    else {index = sprintf(str,"%dKm",int(distance/1000));}
    
    str[index] = 0;
    MyDisplay.drawString(95,44,str);
    MyDisplay.drawCircle(110, 35, 14); 
    if ( blink==true){
     
    MyDisplay.drawCircle(110, 35, 2); 
    blink=false;
    }
    else {
      MyDisplay.drawCircle(110, 35, 4); 
      blink=true;}

    heading=GPSheading(GPS.location.lat(),GPS.location.lng(),LastLat,LastLng);
    x = int(14 * sin(heading));
    y = int(14 * cos(heading));
    MyDisplay.drawLine(110,35, 110+x, 35-y);
    }
    else 
    {
    MyDisplay.setColor(BLACK);
    MyDisplay.fillCircle(110, 35, 15);
    
    }
  }
    CompassUpdate=millis(); 
    MyDisplay.display();
    #endif
  }
  
void OLEDwake()
{
  #ifdef hasLCD
    // Initialising the UI will init the MyDisplay too.
    char str[30];
    MyDisplay.init();
    MyDisplay.flipScreenVertically();
    MyDisplay.clear();
    MyDisplay.setContrast(255);  //contrast is a number between 0 and 255. Use a lower number for lower contrast
    MyDisplay.setColor(WHITE);
    MyDisplay.setFont(ArialMT_Plain_24);
    int index=sprintf(str,"ON");  
    str[index] = 0;
    MyDisplay.drawString(40, 20, str);
    MyDisplay.display();
    MyDelay(500);
    MyDisplay.clear();
    MyDisplay.display();

 #endif
  
}
void OLEDsleep()
{
  #ifdef hasLCD
  char str[30];
  MyDisplay.clear();
  MyDisplay.setContrast(127);  //contrast is a number between 0 and 255. Use a lower number for lower contrast
  MyDisplay.setFont(ArialMT_Plain_24);
  MyDisplay.display();
  MyDelay(100);
   Wire.end(); // stop anymore updates to LCD screen
   VEXToff;
   #endif
 
}

// Service routine
void int64ToChar(byte a[], int64_t n) {
  memcpy(a, &n, 8);
}
void int16ToChar(byte a[], int16_t n) {
  memcpy(a, &n, 2);
}

void floatToChar(byte a[], float f) {
  memcpy(a, &f, sizeof(float));
}



// Service routine


int64_t charTo64bitNum(byte a[]) {
  int64_t n = 0;
  memcpy(&n, a, 8);
  return n;
}

int16_t charTo16bitNum(byte a[]) {
  int16_t n = 0;
  memcpy(&n, a, 2);
  return n;
}

float charTofloatNum(byte a[]) {
  float n = 0;
  memcpy(&n, a, sizeof(float));
  return n;
}

double toRadians(double degrees){
  return degrees *PI/180;
}

void VEXTon(){
    digitalWrite(Vext, LOW); // turn on power to the sensor
    //digitalWrite(GPIO6,LOW);
    VEXstateON=true;
    MyDelay(500); 

}
void VEXToff(){
    MyDelay(500);
    digitalWrite(Vext,HIGH); // turn off power to the sensor
   // digitalWrite(GPIO6,HIGH);
    VEXstateON=false;
    

}

void MyDelay(uint32_t DelayMillis)  // none blocking feeding WDT
{
    uint32_t startMillis =millis();
    while (millis()-startMillis<DelayMillis){
      feedInnerWdt();
      Radio.IrqProcess( ); 
    }

  }

  
  

double GPSheading(double lat1,double lon1,double lat2,double lon2){
  #ifdef hasGPS
    double courseToDestination=GPS.courseTo(lat1,lon1,lat2,lon2);
    const char *directionToDestination=GPS.cardinal(courseToDestination);
    int courseChangeNeeded = (int)(360 + courseToDestination - GPS.course.deg()) % 360;
    if (debugGPS==true){
    Serial.print("DEBUG: Course2Dest: ");
    Serial.print(courseToDestination);
    Serial.print("  CurCourse: ");
    Serial.print(GPS.course.deg());
    Serial.print("  Dir2Dest: ");
    Serial.print(directionToDestination);
    Serial.print("  RelCourse: ");
    Serial.print(courseChangeNeeded);
    Serial.print("  CurSpd: ");
    Serial.println(GPS.speed.kmph());
    }
    //double CourseRads=courseToDestination * (PI/180) ; // convert radians
    double CourseRads=courseChangeNeeded * (PI/180) ; // convert radians
  return CourseRads; 
  #endif
}

void ButstonPress() {
 //Debounce switch
      static unsigned long last_interrupt_time = 0;
      unsigned long interrupt_time = millis();
     // If interrupts come faster than 200ms, assume it's a bounce and ignore
      if (interrupt_time - last_interrupt_time > 200)
        {

          VEXTon();
          if (debugLED==true){
            
          turnOnRGB(0xFF0000,0); //red
          MyDelay(1000);
          turnOffRGB();
          debugLED=false;
          }
          else{
            turnOnRGB(0x00ff00,0); //red
          MyDelay(1000);
          turnOffRGB();
          debugLED=true; 

          }

        }
      last_interrupt_time = interrupt_time;
 }