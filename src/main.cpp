#include <Arduino.h>
//#include <PString.h>
//#include <DHT.h>
#include "EasyNextionLibrary.h"  // Include EasyNextionLibrary
#include <SimpleDHT.h>
#include <Adafruit_DS3502.h>

#define DHTPIN_TOP PB3    // what pin we're connected to
#define DHTPIN_BTM PB12    // what pin we're connected to
#define DHTPIN_FMT PA15    // what pin we're connected to
#define DHTPIN_AMB PB13    // what pin we're connected to
#define DHTPIN_OUT PB5    // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)*/

#define LIGHT_TOP_PWM PB4
#define LIGHT_BTM_PWM PA1

USBSerial Serial0=Serial;


// MAIN ===========================================================
long currentmicros=0;
long currentms=0;
long currentsec=0;
int currentmin=0;
int currentTimeHours;
String uptimeString;

int systemStatusBool;
int lastSystemIncStepTime;
int systemIncStepInt=1000;
bool runSystemIncCommands;

int topHeatEnable=0;
int btmHeatEnable=0;
int fmtHeatEnable=0;

int topHeaterCurrentDuration;
int btmHeaterCurrentDuration;
int fmtHeaterCurrentDuration;

int topHeatActual=0;
int btmHeatActual=0;
int fmtHeatActual=0;

int gTopTempFset=140;
int gBtmTempFset=140;
int gFmtTempFset=140;

int gExhActual=0;
int exhaustControlOutput=0;

int topTimerEnable;
int btmTimerEnable;
int fmtTimerEnable;

float topTimerSet;
int btmTimerSet;
int fmtTimerSet;

float topHeaterInitialOnTime;
float btmHeaterInitialOnTime;
float fmtHeaterInitialOnTime;

float topHeaterActualOnTime;
float btmHeaterActualOnTime;
float fmtHeaterActualOnTime;

//top heater variables to track heating ratio
float topPercentHeatingEff;
bool topHeatIsInitialHeatRamp;
bool topHeatInitialHeatRampComplete;
float topHeaterEnableStart=currentsec;
float topHeaterEnableDuration=0;
float topHeaterActualCycleDuration=0;
float topHeaterActualCycleStart;
float topHeaterActualCycleAdd;

//btm heater variables to track heating ratio
float btmPercentHeatingEff;
bool btmHeatIsInitialHeatRamp;
bool btmHeatInitialHeatRampComplete;
float btmHeaterEnableStart=currentsec;
float btmHeaterEnableDuration=0;
float btmHeaterActualCycleDuration=0;
float btmHeaterActualCycleStart;
float btmHeaterActualCycleAdd;

//fmt heater variables to track heating ratio
float fmtPercentHeatingEff;
bool fmtHeatIsInitialHeatRamp;
bool fmtHeatInitialHeatRampComplete;
float fmtHeaterEnableStart=currentsec;
float fmtHeaterEnableDuration=0;
float fmtHeaterActualCycleDuration=0;
float fmtHeaterActualCycleStart;
float fmtHeaterActualCycleAdd;

float topHeaterTimerOnTime;
float btmHeaterTimerOnTime;
float fmtHeaterTimerOnTime;

bool topHeatIsRunning;
bool btmHeatIsRunning;
bool fmtHeatIsRunning;

bool topHeaterIsShutDown;
bool btmHeaterIsShutDown;
bool fmtHeaterIsShutDown;

int relayHeaterFanOffTimeTop;
int relayHeaterFanOffTimeBtm;
int relayHeaterFanOffTimeFmt;
bool heatWasOnTop;
bool heatWasOnBtm;
bool heatWasOnFmt;

int gRecircTop;
int gRecircBtm;
int gRecircFmt;

const int damperServoBtmOpen=420;
const int damperServoBtmClosed=695;
const int DAMPER_CLOSED=0;
const int DAMPER_LOW=5;
const int DAMPER_MEDLOW=10;
const int DAMPER_MED=50;
const int DAMPER_HIGH=100;

const int DAMPER_OPEN=0;
int damperServoTopState=DAMPER_OPEN;
int damperServoBtmState=DAMPER_OPEN;
int gExhTopLoHiTarget;
int gExhBtmLoHiTarget;
int actualTopDamperPos=0;
int actualBtmDamperPos=0;

const int lightsOffPWM=0;
const int lightsLowPWM=25;
const int lightsMedPWM=135;
const int lightsHighPWM=255;

int gLightsTop=0;
int gLightsBtm=0;
int currentBtmPWM=0;
int currentTopPWM=0;

int ledRampStepInt=1; //millis
int lastLedRampStepTime=0;
int ledLookupTable[] = {0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,7,7,7,8,8,8,9,9,9,10,10,11,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,23,23,24,24,25,26,26,27,28,28,29,30,30,31,32,32,33,34,35,35,36,37,38,38,39,40,41,42,42,43,44,45,46,47,47,48,49,50,51,52,53,54,55,56,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,73,74,75,76,77,78,79,80,81,82,84,85,86,87,88,89,91,92,93,94,95,97,98,99,100,102,103,104,105,107,108,109,111,112,113,115,116,117,119,120,121,123,124,126,127,128,130,131,133,134,136,137,139,140,142,143,145,146,148,149,151,152,154,155,157,158,160,162,163,165,166,168,170,171,173,175,176,178,180,181,183,185,186,188,190,192,193,195,197,199,200,202,204,206,207,209,211,213,215,217,218,220,222,224,226,230,232,234,237,239,242,244,245,247,249,251,255,255,255,255};

// ====  DHT22 SENSORS ========================================
const int NUM_SENSOR_READINGS=5;
struct SensorData{
  float temp=0;
  float hum=0;
  int id; 
  float tempReadings[NUM_SENSOR_READINGS]={0,0,0,0,0};
  float humReadings[NUM_SENSOR_READINGS]={0,0,0,0,0};
  int writeIndex=0;
  float tempAvg=0;
  float humAvg=0;
};

SensorData sensorDataTop {};
SensorData sensorDataBtm {};
SensorData sensorDataFmt {};
SensorData sensorDataAmb {};
SensorData sensorDataOut {};

//int pinDHT22 = PB12;
SimpleDHT22 dhtTop(DHTPIN_TOP);
SimpleDHT22 dhtBtm(DHTPIN_BTM);
SimpleDHT22 dhtFmt(DHTPIN_FMT);
SimpleDHT22 dhtAmb(DHTPIN_AMB);
SimpleDHT22 dhtOut(DHTPIN_OUT);
HardwareSerial Serial2(USART2);   // PA3  (RX)  PA2  (TX)
HardwareSerial Serial3(USART3);// PB11 (RX)  PB10   (TX)

// ==== RELAY ASSIGNMENTS ======================================
const int RELAY_NC=HIGH;
const int RELAY_ON=LOW;
const int RELAY_COUNT=8;
const int RELAY_HTR_FAN_OFF_LIMIT=60; //seconds for heater fan to continue running after it's heater has been shutdown

const int allRelays[RELAY_COUNT]={PC13,PC14,PA0,PA4,PA5,PB8,PB7,PB6};
const int relayHeaterFanFmt=allRelays[7];
const int relayHeaterFanTop=allRelays[1];
const int relayHeaterFanBtm=allRelays[0];
const int relayExhMain=allRelays[2];
const int relayExhLowHigh=allRelays[3];
const int heaterSSRTop=PA7;
const int heaterSSRBtm=PB0;
const int heaterSSRFmt=PB1;
const int heaterLockRLfmt=allRelays[6];
const int heaterLockRLbtm=allRelays[5];
const int heaterLockRLtop=allRelays[4];

EasyNex nexMain(Serial2);
EasyNex nexStatus(Serial1);
// ===================== MAIN FUNCTIONS ======================================================================================
String getCleanString(double p_tempF){
  if(p_tempF>999 || p_tempF<-99)return "ERR";
  char temperatureFTemp[5];
  //PString(temperatureFTemp, sizeof(temperatureFTemp), p_tempF);
  dtostrf(p_tempF, 5, 1, temperatureFTemp);
  return temperatureFTemp;
}
String getCleanStringInt(int p_val){
  if(p_val>999 || p_val<-99)return "ERR";
  char temp[3];
  dtostrf(p_val, 3, 0, temp);
  return temp;
}
String getCleanStringTime(int p_hrs, int p_mins){
  String hourStr=String (p_hrs);
  String minsStr=String (p_mins);
  if(p_hrs<10)hourStr="0"+hourStr;
  if(p_mins<10)minsStr="0"+minsStr;
  return hourStr+":"+minsStr;
}
void shutdownTopHeater(){
  digitalWrite(heaterLockRLtop, HIGH);    // RELAY LOCK ON
  digitalWrite(heaterSSRTop, LOW);    // SSR OFF
  // ^ keep GPIO states above the run once code to ensure the relays remain off when heat called is 0
  if(topHeaterIsShutDown)return;
  //run once code
  topHeaterIsShutDown=true;
  topHeatEnable=0;
  nexMain.writeNum("page0.topHeatEnable.val",0);
  nexMain.writeNum("page0.topTimerEnable.val",0);
  nexStatus.writeStr("page0.gTimer1.txt","");
  topHeatActual=0;
  topHeaterTimerOnTime=0;
  topHeatIsRunning=false;
}
void shutdownBtmHeater(){
  digitalWrite(heaterLockRLbtm, HIGH);    // RELAY LOCK ON
  digitalWrite(heaterSSRBtm, LOW);    // SSR OFF
  // ^ keep GPIO states above the run once code to ensure the relays remain off when heat called is 0
  if(btmHeaterIsShutDown)return;
  //run once code
  btmHeaterIsShutDown=true;
  btmHeatEnable=0;
  nexMain.writeNum("page0.btmHeatEnable.val",0);
  nexMain.writeNum("page0.btmTimerEnable.val",0);
  nexStatus.writeStr("page0.gTimer2.txt","");
  btmHeatActual=0;
  btmHeaterTimerOnTime=0;
  btmHeatIsRunning=false;
}
void shutdownFmtHeater(){
  digitalWrite(heaterLockRLfmt, HIGH);    // RELAY LOCK ON
  digitalWrite(heaterSSRFmt, LOW);    // SSR OFF
  // ^ keep GPIO states above the run once code to ensure the relays remain off when heat called is 0
  if(fmtHeaterIsShutDown)return;
  //run once code
  fmtHeaterIsShutDown=true;
  fmtHeatEnable=0;
  nexMain.writeNum("page0.fmtHeatEnable.val",0);
  nexMain.writeNum("page0.fmtTimerEnable.val",0);
  nexStatus.writeStr("page0.gTimer3.txt","");
  fmtHeatActual=0;
  fmtHeaterTimerOnTime=0;
  fmtHeatIsRunning=false;
}
String getCleanHHMMSS(float timeRemain){
  int secremain=timeRemain;
  int minutesRemain=secremain/60;
  int hrremain=minutesRemain/60;

  String hours=(hrremain<10)? "0"+String(hrremain) : String(hrremain);
  int minConcat= minutesRemain%60;
  String mins=(minConcat<10)? "0"+String(minConcat) : String(minConcat);
  int secConcat = secremain%60;
  String secs=(secConcat<10)? "0"+String(secConcat) : String(secConcat);
  String remainingString=hours+":"+mins+":"+secs;
  return remainingString;
}
String getCleanMMSS(float timeRemain){
  int secremain=timeRemain;
  int minutesRemain=secremain/60;
  int hrremain=minutesRemain/60;

  int minConcat= minutesRemain%60;
  String mins=(minConcat<10)? "0"+String(minConcat) : String(minConcat);
  int secConcat = secremain%60;
  String secs=(secConcat<10)? "0"+String(secConcat) : String(secConcat);
  String remainingString=mins+":"+secs;
  return remainingString;
}
String getCleanSS(float timeRemain){
  int secremain=timeRemain;
  int secConcat = secremain%60;
  String secs=(secConcat<10)? "0"+String(secConcat) : String(secConcat);
  String remainingString=secs;
  return remainingString;
}

void heaterCheck(){
  // ======= TOP HEATER =================== //
  if(topHeatEnable==1){
    heatWasOnTop=true;
    if(!topHeatIsRunning){
      topHeatIsInitialHeatRamp=true;
      topHeatInitialHeatRampComplete=false;
      topHeatIsRunning=true;
      topHeaterIsShutDown=false;
      topPercentHeatingEff=100;
      digitalWrite(heaterLockRLtop, LOW);    // RELAY LOCK OFF
    }
    if(!topHeatIsInitialHeatRamp && !topHeatInitialHeatRampComplete){
      topHeatInitialHeatRampComplete=true;
      //
      topHeaterEnableStart=currentsec;
      topHeaterEnableDuration=0;
      topHeaterActualCycleDuration=0;
      topHeaterActualCycleAdd=0;
      topHeaterActualCycleStart=currentsec;
      topHeaterActualOnTime=0;
    }
    
    if((topTimerEnable) && (currentsec-topHeaterTimerOnTime>=(topTimerSet*60*60))){
      shutdownTopHeater();
      return;
    }
    if(sensorDataTop.temp<gTopTempFset){
      if(!topHeatActual){
        topHeaterActualCycleStart=currentsec;
        topHeaterActualCycleAdd=currentsec;
        digitalWrite(heaterSSRTop, HIGH);    // SSR ON
        topHeatActual=1;
      }
    }else{
      topHeatIsInitialHeatRamp=false;
      if(topHeatActual){
        digitalWrite(heaterSSRTop, LOW);    // SSR OFF
        topHeatActual=0;
      }
    }
    if(topHeatActual){
      topHeaterActualOnTime+=(currentsec-topHeaterActualCycleAdd);
      topHeaterActualCycleAdd=currentsec;
    }
    topHeaterEnableDuration=currentsec-topHeaterEnableStart;
    topPercentHeatingEff=100-((topHeaterActualOnTime/topHeaterEnableDuration)*100);
  }else{
    shutdownTopHeater();
    if(heatWasOnTop){
      relayHeaterFanOffTimeTop=currentsec;
      heatWasOnTop=false;
    }
  }
  // ======= BOTTOM HEATER =================== //
  if(btmHeatEnable==1){
    heatWasOnBtm=true;
    if(!btmHeatIsInitialHeatRamp && !btmHeatInitialHeatRampComplete){
      btmHeatInitialHeatRampComplete=true;
      //
      btmHeaterEnableStart=currentsec;
      btmHeaterEnableDuration=0;
      btmHeaterActualCycleDuration=0;
      btmHeaterActualCycleAdd=0;
      btmHeaterActualCycleStart=currentsec;
      btmHeaterActualOnTime=0;
    }
    if(!btmHeatIsRunning){
      btmHeatIsInitialHeatRamp=true;
      btmHeatInitialHeatRampComplete=false;
      btmHeatIsRunning=true;
      btmHeaterIsShutDown=false;
      btmPercentHeatingEff=100;
      digitalWrite(heaterLockRLbtm, LOW);    // RELAY LOCK OFF
    }
    
    if((btmTimerEnable) && (currentsec-btmHeaterTimerOnTime>=(btmTimerSet*60*60))){
      shutdownBtmHeater();
      return;
    }
    if(sensorDataBtm.temp<gBtmTempFset){
      if(!btmHeatActual){
        btmHeaterActualCycleStart=currentsec;
        btmHeaterActualCycleAdd=currentsec;
        digitalWrite(heaterSSRBtm, HIGH);    // SSR ON
        btmHeatActual=1;
      }
    }else{
      btmHeatIsInitialHeatRamp=false;
      if(btmHeatActual){
        digitalWrite(heaterSSRBtm, LOW);    // SSR OFF
        btmHeatActual=0;
      }
    }
    if(btmHeatActual){
      btmHeaterActualOnTime+=(currentsec-btmHeaterActualCycleAdd);
      btmHeaterActualCycleAdd=currentsec;
    }
    btmHeaterEnableDuration=currentsec-btmHeaterEnableStart;
    btmPercentHeatingEff=100-((btmHeaterActualOnTime/btmHeaterEnableDuration)*100);
  }else{
    shutdownBtmHeater();
    if(heatWasOnBtm){
      relayHeaterFanOffTimeBtm=currentsec;
      heatWasOnBtm=false;
    }
  }

  // ======= FILAMENT HEATER =================== //
  if(fmtHeatEnable==1){
    heatWasOnFmt=true;
    if(!fmtHeatIsRunning){
      fmtHeatIsInitialHeatRamp=true;
      fmtHeatInitialHeatRampComplete=false;
      fmtHeatIsRunning=true;
      fmtHeaterIsShutDown=false;
      fmtPercentHeatingEff=0;
      digitalWrite(heaterLockRLfmt, LOW);    // RELAY LOCK OFF
    }
    if(!fmtHeatIsInitialHeatRamp && !fmtHeatInitialHeatRampComplete){
      fmtHeatInitialHeatRampComplete=true;
      //
      fmtHeaterEnableStart=currentsec;
      fmtHeaterEnableDuration=0;
      fmtHeaterActualCycleDuration=0;
      fmtHeaterActualCycleAdd=0;
      fmtHeaterActualCycleStart=currentsec;
      fmtHeaterActualOnTime=0;
    }
    if((fmtTimerEnable) && (currentsec-fmtHeaterTimerOnTime>=(fmtTimerSet*60*60))){
      shutdownFmtHeater();
      return;
    }
    if(sensorDataFmt.temp<gFmtTempFset){
      if(!fmtHeatActual){
        fmtHeaterActualCycleStart=currentsec;
        fmtHeaterActualCycleAdd=currentsec;
        digitalWrite(heaterSSRFmt, HIGH);    // SSR ON
        fmtHeatActual=1;
      }
    }else{
      fmtHeatIsInitialHeatRamp=false;
      if(fmtHeatActual){
        digitalWrite(heaterSSRFmt, LOW);    // SSR OFF
        fmtHeatActual=0;
      }
    }
    if(fmtHeatActual){
      fmtHeaterActualOnTime+=(currentsec-fmtHeaterActualCycleAdd);
      fmtHeaterActualCycleAdd=currentsec;
    }
    fmtHeaterEnableDuration=currentsec-fmtHeaterEnableStart;
    fmtPercentHeatingEff=100-(fmtHeaterActualOnTime/fmtHeaterEnableDuration)*100;
  }else{
    shutdownFmtHeater();
    if(heatWasOnFmt){
      relayHeaterFanOffTimeFmt=currentsec;
      heatWasOnFmt=false;
    }
  }
}
/*void sendStringToNextion(HardwareSerial &p_device,String p_var, String p_data){
  p_var.concat("=\"");
  p_var.concat(p_data);
  p_var.concat("\"");
  for (int i = 0; i < p_var.length(); i++)
  {
    p_device.write(p_var[i]);   // Push each char 1 by 1 on each loop pass  
  }
  p_device.write(0xff); //We need to write the 3 ending bits to the Nextion as well
  p_device.write(0xff); //it will tell the Nextion that this is the end of what we want to send.
  p_device.write(0xff);
}*/

void setRelayModes(){
  for (int i = 0; i < RELAY_COUNT; i++) {
    if(allRelays[i])pinMode(allRelays[i], OUTPUT);
  }
  pinMode(heaterSSRTop, OUTPUT);
  pinMode(heaterSSRBtm, OUTPUT);
  pinMode(heaterSSRFmt, OUTPUT);
}
void allRelaysNC(){
  for (int i = 0; i < RELAY_COUNT; i++) {
    if(allRelays[i])digitalWrite(allRelays[i], RELAY_NC);
  }
  pinMode(heaterSSRTop, RELAY_ON);
  pinMode(heaterSSRBtm, RELAY_ON);
  pinMode(heaterSSRFmt, RELAY_ON);
}
void allRelaysON(){
  for (int i = 0; i < RELAY_COUNT; i++) {
    if(allRelays[i])digitalWrite(allRelays[i], RELAY_ON);
  }
  pinMode(heaterSSRTop, RELAY_NC);
  pinMode(heaterSSRBtm, RELAY_NC);
  pinMode(heaterSSRFmt, RELAY_NC);
}
void setup() {
  sensorDataTop.id=101;
  sensorDataBtm.id=102;
  sensorDataFmt.id=103;
  sensorDataAmb.id=104;
  sensorDataOut.id=105;
  pinMode(LIGHT_TOP_PWM,OUTPUT);
  pinMode(LIGHT_BTM_PWM,OUTPUT);
  pinMode(PA0, INPUT_ANALOG);
  nexMain.begin(57600);
  nexStatus.begin(57600);
  Serial3.begin(9600);
  delay(100);
  setRelayModes();
  serialEventRun();
}

void readDHT(SimpleDHT22 p_sensor, SensorData &dataTarget){
  byte data[40] = {0};
  //float oldTempReading=dataTarget.temp;
  float newTempReading;
  float newHumReading;
  if(p_sensor.read2(&newTempReading, &newHumReading, data)!=0){
    int idnum=dataTarget.id;
    // ERROR HERE if 0
  }
  newTempReading=newTempReading* 9 / 5 + 32;
  //if(newTempReading!=NAN && abs(newTempReading-oldTempReading)>0.2)dataTarget.temp=newTempReading;
  if(newTempReading!=NAN){
    dataTarget.temp=newTempReading;
    dataTarget.tempReadings[dataTarget.writeIndex]=newTempReading;
  }
  if(newHumReading!=NAN){
    dataTarget.hum=newHumReading;
    dataTarget.humReadings[dataTarget.writeIndex]=newHumReading;
  }
  float sum=0;
  for (size_t i = 0; i < NUM_SENSOR_READINGS; i++){
    sum+=dataTarget.tempReadings[i];
  }
  dataTarget.tempAvg=sum/NUM_SENSOR_READINGS;  
  
  sum=0;
  for (size_t i = 0; i < NUM_SENSOR_READINGS; i++){
    sum+=dataTarget.humReadings[i];
  }
  dataTarget.humAvg=sum/NUM_SENSOR_READINGS;  

  dataTarget.writeIndex++;
  if (dataTarget.writeIndex==NUM_SENSOR_READINGS)dataTarget.writeIndex=0;
}


void readAtmosData(){
  readDHT(dhtTop,sensorDataTop);
  readDHT(dhtBtm,sensorDataBtm);
  readDHT(dhtFmt,sensorDataFmt);
  readDHT(dhtAmb,sensorDataAmb);
  readDHT(dhtOut,sensorDataOut);
}
float float_one_point_round(float value)
{
        return ((float)((int)(value * 10))) / 10;
}
// ======================= SEND DATA OUT ======================================================================
void sendAtmosDataToNexDisplay(){
  // TEMPS
  String temp=getCleanString(sensorDataTop.tempAvg);

  nexStatus.writeStr("page0.gTopTempF.txt", temp);

  temp=getCleanString(sensorDataBtm.tempAvg);
  nexStatus.writeStr("page0.gBtmTempF.txt", temp);

  temp=getCleanString(sensorDataFmt.tempAvg);
  nexStatus.writeStr("page0.gFmtTempF.txt", temp);

  temp=getCleanString(sensorDataOut.tempAvg);
  nexStatus.writeStr("page0.gOutTempF.txt", temp);
 
  temp=getCleanString(sensorDataAmb.tempAvg);
  nexStatus.writeStr("page0.gAmbTempF.txt", temp);

  //HUMIDITY
  char humStr[5];
  if(!topHeatEnable){
    dtostrf(sensorDataTop.humAvg, 5, 1, humStr);
    nexStatus.writeStr("page0.gTopRH.txt", "R/H:"+String(humStr)+"%");
  }else{
    if(topHeatIsInitialHeatRamp){
      nexStatus.writeStr("page0.gTopRH.txt", "Heating");
    }else{
      String textVal= String(float_one_point_round(topPercentHeatingEff));
      textVal.remove(textVal.length()-1,1);
      nexStatus.writeStr("page0.gTopRH.txt", "Eff:"+textVal+"%");
    }
  }

  if(!btmHeatEnable){
    dtostrf(sensorDataBtm.humAvg, 5, 1, humStr);
    nexStatus.writeStr("page0.gBtmRH.txt", "R/H:"+String(humStr)+"%");
  }else{
    if(btmHeatIsInitialHeatRamp){
      nexStatus.writeStr("page0.gBtmRH.txt", "Heating");
    }else{
      String textVal= String(float_one_point_round(btmPercentHeatingEff));
      textVal.remove(textVal.length()-1,1);
      nexStatus.writeStr("page0.gBtmRH.txt", "Eff:"+textVal+"%");
    }
  }

  if(!fmtHeatEnable){
    dtostrf(sensorDataOut.humAvg, 5, 1, humStr);
    nexStatus.writeStr("page0.gOutRH.txt", "R/H:"+String(humStr)+"%");
  }else{
    if(fmtHeatIsInitialHeatRamp){
      nexStatus.writeStr("page0.gOutRH.txt", "Heating");
    }else{
      String textVal= String(float_one_point_round(fmtPercentHeatingEff));
      textVal.remove(textVal.length()-1,1);
      nexStatus.writeStr("page0.gOutRH.txt", "Eff:"+textVal+"%");
    }
  }
  

  dtostrf(sensorDataAmb.humAvg, 5, 1, humStr);
  nexStatus.writeStr("page0.gAmbRH.txt", "R/H:"+String(humStr)+"%");

  dtostrf(sensorDataFmt.humAvg, 4, 1, humStr);
  String newHum=humStr;
  nexStatus.writeStr("page0.gRhActual.txt", newHum);
}

void sendDataNexMain(){
  nexMain.writeStr("page0.uptimeString.txt", uptimeString);
  nexMain.writeNum("page0.dmperTopPos.val", actualTopDamperPos);
}
void sendDataNexStatus(){
  nexStatus.writeNum("page0.topHeatEnable.val", topHeatEnable); // 
  nexStatus.writeNum("page0.btmHeatEnable.val", btmHeatEnable); // 
  nexStatus.writeNum("page0.fmtHeatEnable.val", fmtHeatEnable); // 
  nexStatus.writeNum("page0.gTopHtActual.val", topHeatActual); // feedback to show if heater actually is on
  nexStatus.writeNum("page0.gBtmHtActual.val", btmHeatActual); // feedback to show if heater actually is on
  nexStatus.writeNum("page0.gFmtHtActual.val", fmtHeatActual); // feedback to show if heater actually is on
  nexStatus.writeNum("page0.gExhActual.val", exhaustControlOutput); // actual exhaust speed being called for
  if(sensorDataFmt.hum>=30.0){
    nexStatus.writeNum("page0.gRHwarning.val", 1);
  }else{
    nexStatus.writeNum("page0.gRHwarning.val", 0);
  }
  nexStatus.writeStr("page0.gTopTempFset.txt", getCleanStringInt(gTopTempFset)); //
  nexStatus.writeStr("page0.gBtmTempFset.txt", getCleanStringInt(gBtmTempFset)); //
  nexStatus.writeStr("page0.gFmtTempFset.txt", getCleanStringInt(gFmtTempFset)); //
  nexStatus.writeStr("page0.tDmpTop.txt", String(actualTopDamperPos)+"%");

  nexStatus.writeStr("page0.tDmpBtm.txt", "100%"); 
  nexStatus.writeStr("page0.tDmpAmb.txt", "--"); 
}
// =======================================================================================
void topTimerReset(){
  if(topHeatIsRunning)topHeaterTimerOnTime=currentsec;
  nexStatus.writeStr("page0.gTimer1.txt","");
  nexStatus.writeNum("page0.gTimer1vis.val",0);
}
void btmTimerReset(){
  if(btmHeatIsRunning)btmHeaterTimerOnTime=currentsec;
  nexStatus.writeStr("page0.gTimer2.txt","");
  nexStatus.writeNum("page0.gTimer2vis.val",0);
}
void fmtTimerReset(){
  if(fmtHeatIsRunning)fmtHeaterTimerOnTime=currentsec;
  nexStatus.writeStr("page0.gTimer3.txt","");
  nexStatus.writeNum("page0.gTimer3vis.val",0);
}
void getnexMainVariables(){
  topHeatEnable = nexMain.readNumber("page0.topHeatEnable.val");   // Pull the top heater enabled variable
  btmHeatEnable = nexMain.readNumber("page0.btmHeatEnable.val");   // Pull the btm heater enabled variable
  fmtHeatEnable = nexMain.readNumber("page0.fmtHeatEnable.val");   // Pull the fmt heater enabled variable
  gTopTempFset = nexMain.readNumber("page0.gTopTempFset.val");   // Pull the top heater temp set
  gBtmTempFset = nexMain.readNumber("page0.gBtmTempFset.val");   // Pull the btm heater temp set
  gFmtTempFset = nexMain.readNumber("page0.gFmtTempFset.val");   // Pull the btm heater temp set
  gExhActual = nexMain.readNumber("page0.gExhActual.val");   // Pull the exhaust fan val 0-2
  gLightsTop = nexMain.readNumber("page0.gLightsTop.val");   // Pull the lights val 0-3
  gLightsBtm = nexMain.readNumber("page0.gLightsBtm.val");   // Pull the lights val 0-3
  gExhTopLoHiTarget=nexMain.readNumber("page0.gExhTopLoHi.val");   // Pull the btm exh target 0/1
  gExhBtmLoHiTarget=nexMain.readNumber("page0.gExhBtmLoHi.val");   // Pull the btm exh target 0/1

  gRecircTop=nexMain.readNumber("page0.gRecircTop.val");
  gRecircBtm=nexMain.readNumber("page0.gRecircBtm.val");
  gRecircFmt=nexMain.readNumber("page0.gRecircFmt.val");

  int oldTopTimerEnable=topTimerEnable;
  topTimerEnable = nexMain.readNumber("page0.topTimerEnable.val");
  if(topTimerEnable!=oldTopTimerEnable)topTimerReset();

  int oldBtmTimerEnable=btmTimerEnable;
  btmTimerEnable = nexMain.readNumber("page0.btmTimerEnable.val");
  if(btmTimerEnable!=oldBtmTimerEnable)btmTimerReset();

  int oldFmtTimerEnable=fmtTimerEnable;
  fmtTimerEnable = nexMain.readNumber("page0.fmtTimerEnable.val");
  if(fmtTimerEnable!=oldFmtTimerEnable)fmtTimerReset();
  
  topTimerSet=float(nexMain.readNumber("page0.topTimerSet.val"));
  btmTimerSet=float(nexMain.readNumber("page0.btmTimerSet.val"));
  fmtTimerSet=float(nexMain.readNumber("page0.fmtTimerSet.val"));
}
// ==== MAIN STATUS LED BLINK

void systemStatus(){
  systemStatusBool=!systemStatusBool;
  if(systemStatusBool){
    //digitalWrite(led, HIGH);
    //allRelaysNC();
  }
  if(!systemStatusBool){
    //digitalWrite(led, LOW);
    //allRelaysON();
  }
  Serial0.println("Status: OK Serial");
  //Serial0.println(uptimeString);
  //Serial1.println("Status: OK Serial1");
  //Serial2.println("Status: OK Serial2");
  //Serial3.println("Status: OK Serial3");
  /*
  Serial0.print(" | tTop: ");
  Serial0.print(sensorDataTop[0]);
  Serial0.print(" | hTop: ");
  Serial0.println(sensorDataTop[1]);
  Serial0.print(" | tBtm: ");
  Serial0.print(sensorDataBtm[0]);
  Serial0.print(" | hTBtm: ");
  Serial0.println(sensorDataBtm[1]);
  Serial0.print(" | tFmt: ");
  Serial0.print(sensorDataFmt[0]);
  Serial0.print(" | hFmt: ");
  Serial0.println(sensorDataFmt[1]);
  Serial0.print(" | tAmb: ");
  Serial0.print(sensorDataAmb[0]);
  Serial0.print(" | hAmb: ");
  Serial0.println(sensorDataAmb[1]);
  Serial0.print(" | tOut: ");
  Serial0.print(sensorDataOut[0]);
  Serial0.print(" | hOut: ");
  Serial0.println(sensorDataOut[1]);*/
  //Serial3.println(35);
}

void damperControl(){
  int topOverridesForLOW=0;
  int topOverridesForHIGH=0;
  int btmOverridesForLOW=0;
  int btmOverridesForHIGH=0;
  actualTopDamperPos=DAMPER_HIGH; // if all else fails just default to open
  actualBtmDamperPos=DAMPER_HIGH; // if all else fails just default to open
  switch(gExhTopLoHiTarget){
    case 0://Damper OFF
      actualTopDamperPos=0;
    break;
    case 1://Damper LOW
      if(gExhActual==0){
        actualTopDamperPos=DAMPER_CLOSED;
      }else if(gExhActual==1){//MAIN ON AUTO
        topOverridesForLOW=1;//force exhaustControlOutput speed to 1 = LOW
        if(!topHeatEnable){
          actualTopDamperPos=DAMPER_HIGH; // NO HEAT, KEEP AS COOL AS POSSIBLE ON LOW FAN
        }else{
          actualTopDamperPos=DAMPER_MEDLOW; // HEAT ON, don't pull too much air too fast, even on low fan
        }
        if(gExhBtmLoHiTarget==2){
          actualTopDamperPos=DAMPER_LOW;
        }
         
      }else{//MAIN ON HIGH
        actualTopDamperPos=DAMPER_LOW;
      }      
    break;
    case 2://Damper HIGH
      if(gExhActual==0){
        actualTopDamperPos=DAMPER_CLOSED;
      }else if(gExhActual==1){//MAIN ON AUTO
        topOverridesForHIGH=1;//force exhaustControlOutput speed to 2 = HIGH
        actualTopDamperPos=DAMPER_HIGH;
      }else{//MAIN ON HIGH
        actualTopDamperPos=DAMPER_HIGH;
      }    
    break;
  }
  switch(gExhBtmLoHiTarget){
    case 0://Damper OFF
      //Serial3.println(0);
      //damperServoBtmState=DAMPER_CLOSED; 
    break;
    case 1://Damper LOW
      if(gExhActual==0){
        //Serial3.println(0);
        //damperServoBtmState=DAMPER_CLOSED;
      }else if(gExhActual==1){//MAIN ON AUTO
        //btmOverridesForLOW=1;//force exhaustControlOutput speed to 1 = LOW
        //Serial3.println(30);
        //damperServoBtmState=DAMPER_CLOSED;
      }else{//MAIN ON HIGH
        //Serial3.println(7);
        //damperServoBtmState=DAMPER_CLOSED;
      }      
    break;
    case 2://Damper HIGH
      if(gExhActual==0){
        //Serial3.println(0);
      }else if(gExhActual==1){//MAIN ON AUTO
        //btmOverridesForHIGH=1;//force exhaustControlOutput speed to 2 = HIGH
        //Serial3.println(100);
        //damperServoBtmState=DAMPER_OPEN;
      }else{//MAIN ON HIGH
        //Serial3.println(100);
        //damperServoBtmState=DAMPER_OPEN;
      }    
    break;
  }

  Serial3.println(actualTopDamperPos);
  
  if(topOverridesForLOW || btmOverridesForLOW)exhaustControlOutput=1;
  if(topOverridesForHIGH || btmOverridesForHIGH)exhaustControlOutput=2;
  if(gExhActual==1 && gExhTopLoHiTarget==0 && gExhBtmLoHiTarget==0){

    exhaustControlOutput=0;
  }
  // switch case above, damper controls will turn exhaustControlOutput to 1 if either need it
  switch(gExhActual){
    case 0: // OVERRIDE OFF
      exhaustControlOutput=0;
    break;
    case 2: // AUTO
      exhaustControlOutput=2;
    break;
  }

  //if(damperServoBtmState==DAMPER_OPEN)analogWrite(damperServoBtmPin, damperServoBtmOpen);
  //if(damperServoBtmState==DAMPER_CLOSED)analogWrite(damperServoBtmPin, damperServoBtmClosed);
  //Serial0.println(actualTopDamperPos);

  switch (exhaustControlOutput){
    case 0:
      digitalWrite(relayExhMain, RELAY_ON); //ON shuts off FAN, normally closed defaults to ON/LOW
      digitalWrite(relayExhLowHigh, RELAY_NC);
      break;
    case 1:
      digitalWrite(relayExhMain, RELAY_NC);
      digitalWrite(relayExhLowHigh, RELAY_NC);
      break;
    case 2:
      digitalWrite(relayExhMain, RELAY_NC);
      digitalWrite(relayExhLowHigh, RELAY_ON);
      break;
  }
}
void showNexElement(String p_element){
  Serial1.print("vis "+p_element+",1");
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}
void hideNexElement(String p_element){
  Serial1.print("vis "+p_element+",0");
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}
// ====== MAIN FAN RECIRCULATOR CHECKS ===============================================================================
bool topInHeaterCooldownMode(){
  if(!topHeatEnable && !gRecircTop && (currentsec-relayHeaterFanOffTimeTop<RELAY_HTR_FAN_OFF_LIMIT))return true;
  return false;
}
bool btmInHeaterCooldownMode(){
  if(!btmHeatEnable && !gRecircBtm && (currentsec-relayHeaterFanOffTimeBtm<=RELAY_HTR_FAN_OFF_LIMIT))return true;
  return false;
}
bool fmtInHeaterCooldownMode(){
  if(!fmtHeatEnable && !gRecircFmt && (currentsec-relayHeaterFanOffTimeFmt<=RELAY_HTR_FAN_OFF_LIMIT))return true;
  return false;
}

void circulatorCheck(){
  int topEnable=0;
  int btmEnable=0;
  int fmtEnable=0;

  // heater & timer logic ------------------------------
  if(topInHeaterCooldownMode())topEnable=1;
  if(btmInHeaterCooldownMode())btmEnable=1;
  if(fmtInHeaterCooldownMode())fmtEnable=1;

  //recirculator overrides if calling for MAX venting ----------------------------
  if(gExhTopLoHiTarget==2 && gExhActual>0)topEnable=1;
  if(gExhBtmLoHiTarget==2 && gExhActual>0)btmEnable=1;
  
  // final heater overrides - if heaters are on, the recirculator fans need to be on
  if(topHeatEnable || gRecircTop)topEnable=1;
  if(btmHeatEnable || gRecircBtm)btmEnable=1;
  if(fmtHeatEnable || gRecircFmt)fmtEnable=1;

  if(topHeatEnable){
    showNexElement("pTopHtFan");
  }else{
    hideNexElement("pTopHtFan");
  }
  if(btmHeatEnable){
    showNexElement("pBtmHtFan");
  }else{
    hideNexElement("pBtmHtFan");
  }
  if(fmtHeatEnable){
    showNexElement("pFmtHtFan");
  }else{
    hideNexElement("pFmtHtFan");
  }
  if(gRecircTop){
    showNexElement("pTopRecFan");
  }else{
    hideNexElement("pTopRecFan");
  }
  if(gRecircBtm){
    showNexElement("pBtmRecFan");
  }else{
    hideNexElement("pBtmRecFan");
  }
  if(gRecircFmt){
    showNexElement("pFmtRecFan");
  }else{
    hideNexElement("pFmtRecFan");
  }

  //final state change --------------------------------
  if(topEnable){
    digitalWrite(relayHeaterFanTop, RELAY_NC); // ON
  }else{
    digitalWrite(relayHeaterFanTop, RELAY_ON); // OFF
  }
  if(btmEnable){
    digitalWrite(relayHeaterFanBtm, RELAY_NC); // ON
  }else{
    digitalWrite(relayHeaterFanBtm, RELAY_ON); // OFF
  }
  if(fmtEnable){
    digitalWrite(relayHeaterFanFmt, RELAY_NC); // HIGH
  }else{
    digitalWrite(relayHeaterFanFmt, RELAY_ON); // LOW
  }
}

void timersCheck(){
  bool showTop=false;
  bool showBtm=false;
  bool showFmt=false;
  String textTop="";
  String textBtm="";
  String textFmt="";
  // TOP COOL DOWN
  if(topInHeaterCooldownMode()){
    float timeRemain=relayHeaterFanOffTimeTop+RELAY_HTR_FAN_OFF_LIMIT-currentsec;
    textTop="COOL: "+getCleanSS(timeRemain);
    showTop=true;
  }
  // TOP HEAT TIMER
  if(topHeatEnable && topTimerEnable){
    float timeRemain=(topTimerSet*60*60) + topHeaterTimerOnTime - currentsec;
    if(timeRemain>0){
      textTop=getCleanHHMMSS(timeRemain);
      showTop=true;
    }else{
      showTop=false;

    }
  }

  if(showTop){
    nexStatus.writeNum("page0.gTimer1vis.val",1);
    nexStatus.writeStr("page0.gTimer1.txt",textTop);
  }else{
    nexStatus.writeStr("page0.gTimer1.txt","");
    nexStatus.writeNum("page0.gTimer1vis.val",0);
  }
  
  // BTM 
  if(btmInHeaterCooldownMode()){
    float timeRemain=relayHeaterFanOffTimeBtm+RELAY_HTR_FAN_OFF_LIMIT-currentsec;
    textBtm="COOL: "+getCleanSS(timeRemain);
    showBtm=true;
  }
  if(btmHeatEnable && btmTimerEnable){ // standard timer enabled, 1-24 hours
    float timeRemain=(btmTimerSet*60*60) + btmHeaterTimerOnTime - currentsec;
    textBtm=getCleanHHMMSS(timeRemain);
    showBtm=true;
  }
  if(showBtm){
    nexStatus.writeNum("page0.gTimer2vis.val",1);
    nexStatus.writeStr("page0.gTimer2.txt",textBtm);
  }else{
    nexStatus.writeStr("page0.gTimer2.txt","");
    nexStatus.writeNum("page0.gTimer2vis.val",0);
  }
  // FMT 
  if(fmtInHeaterCooldownMode()){
    float timeRemain=relayHeaterFanOffTimeFmt+RELAY_HTR_FAN_OFF_LIMIT-currentsec;
    textFmt="COOL: "+getCleanSS(timeRemain);
    showFmt=true;
  }
  if(fmtHeatEnable && fmtTimerEnable){ // standard timer enabled, 1-24 hours
    float timeRemain=(fmtTimerSet*60*60) + fmtHeaterTimerOnTime - currentsec;
    textFmt=getCleanHHMMSS(timeRemain);
    showFmt=true;
  }
  if(showFmt){
    nexStatus.writeNum("page0.gTimer3vis.val",1);
    nexStatus.writeStr("page0.gTimer3.txt",textFmt);
  }else{
    nexStatus.writeStr("page0.gTimer3.txt","");
    nexStatus.writeNum("page0.gTimer3vis.val",0);
  }
}

//nextion state change handler ===============================================================================================

void nextionStateChange(){
    getnexMainVariables();
}

// ==== LIGHTS ================================================

void checkLights(){
  if(currentms<lastLedRampStepTime+ledRampStepInt)return;
  lastLedRampStepTime=currentms;
  int targetTopState;
  int targetBtmState;
  switch(gLightsTop){
    case 0:
      targetTopState=lightsOffPWM;
      break;
    case 1:
      targetTopState=lightsLowPWM;
      break;
    case 2:
      targetTopState=lightsMedPWM;
      break;
    case 3:
      targetTopState=lightsHighPWM;
      break;
  }
  switch(gLightsBtm){
    case 0:
      targetBtmState=lightsOffPWM;
      break;
    case 1:
      targetBtmState=lightsLowPWM;
      break;
    case 2:
      targetBtmState=lightsMedPWM;
      break;
    case 3:
      targetBtmState=lightsHighPWM;
      break;
  }

  if(currentTopPWM<targetTopState){
      currentTopPWM++;
  }else if(currentTopPWM>targetTopState){
    currentTopPWM--;
  }
  analogWrite(LIGHT_TOP_PWM,ledLookupTable[currentTopPWM]);

  if(currentBtmPWM<targetBtmState){
      currentBtmPWM++;
  }else if(currentBtmPWM>targetBtmState){
    currentBtmPWM--;
  }
  analogWrite(LIGHT_BTM_PWM,ledLookupTable[currentBtmPWM]);
}
void systemIncLoop(){
  if(currentms<lastSystemIncStepTime+systemIncStepInt)return;
  lastSystemIncStepTime=currentms;
  getnexMainVariables();
  readAtmosData();
  heaterCheck();
  damperControl();
  circulatorCheck();
  timersCheck();
  String hours=(currentTimeHours<10)? "0"+String(currentTimeHours) : String(currentTimeHours);
  int minConcat= currentmin%60;
  String mins=(minConcat<10)? "0"+String(minConcat) : String(minConcat);
  int secConcat = currentsec%60;
  String secs=(secConcat<10)? "0"+String(secConcat) : String(secConcat);
  uptimeString=hours+":"+mins+":"+secs;
  sendAtmosDataToNexDisplay();
  sendDataNexMain();
  sendDataNexStatus();
  systemStatus();
}
const String STATE_CHANGE="state!ch";
String serial2EventString="";
void serialEvent2() {
  while (Serial2.available()) {
    //char inChar = (char)Serial2.read();
    //serial2EventString+=inChar;
    Serial0.print(Serial2.read());
    if(serial2EventString==STATE_CHANGE){
      //nextionStateChange();
      //Serial0.println("serial2EventString: Serial2");
      
    }
    serial2EventString="";
  }
  //Serial0.println(serial2EventString);
}
void loop() {
  //currentmicros=micros();
  currentms=millis();
  currentsec=currentms/1000;
  currentmin=currentsec/60;
  currentTimeHours = float(currentms)/1000/3600;
  systemIncLoop();//run all commands on the system inc loop time
  checkLights();
  if (Serial2.available())serialEvent2();
}
