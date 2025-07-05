#include <Arduino.h>
#include <permaDefs.h>
#include <SPI.h>
#include <Wire.h>
#include <time.h>
#include <TimeLib.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SPIMemory.h>
#include <elapsedMillis.h>
#include <avr/sleep.h>
#include <MCP79412RTC.h>
#include <DFRobot_LIS2DW12.h>
#include <MPR121.h>

//*** Library Declarations ***//
elapsedMillis mTime;
TinyGPSPlus gps;
SPIFlash flash(FCS);
MCP79412RTC rtc(false);
DFRobot_LIS2DW12_I2C acce;

//#####################################################################################################//

//*** Variables ***//

// General Variables //
uint16_t cnt;                     //** Total data points collected

// Flash Addresses // 
uint32_t wAdd = 0;                //** Last written to flash address
uint32_t rAdd = 0;                //** Last read from flash address

// GPS Control Variables //
int gpsFrequency = 10;            //(Minutes)>>> GPS Frequency in minutes *** USER CONFIG ***
int gpsTimeout = 50;              //(Seconds)>>> GPS Timesout after 'x' seconds *** USER CONFIG ***
int gpsHdop = 5;                  //(N/A)>>> GPS HODP Parameter *** USER CONFIG ***

// GPS Storage Variables //
float lat;                        //** Storing last known Latitude
float lng;                        //** Storing last known Longitude

// Time Variables // 
time_t strtTime = 1667704472;     //** Start Time Acquired by GPS on Start Up
time_t next_gps_wakeup;           //** Internal GPS Time Setting
time_t next_ping_wakeup;           //** Internal Ping Time Setting

// Booleans and Flags //
bool rtc_int_triggered = false;   //** Flag to identify rtc timer trigger 
bool mpr_int_triggered = false;   //** Flag to identify MPR timer trigger
bool wipe_memory = false;         //>>> Wipe memory during reset/Activation
bool activation_resp_rcvd = false;//** Flag to mark if activation commnd was rcvd
bool gps_wait_time = false;

// Radio Variables //
int radioFrequency = 1;           //Frequency of Pings in minutes *** USER CONFIG ***
int rcv_duration = 3;             // Receive Window Duration in seconds *** USER CONFIG ***

// Electrode Status //
bool E1 = true;
bool E2 = true;
bool E3 = true;
bool E4 = true;
bool E5 = true;
bool E6 = true;
bool surface = false;

// MPR Variables //
const uint8_t MPR121_ADDR = 0x5A;
const uint8_t MPR121_INT = 2;
uint8_t touch_trsh = 75;          // Touch Threshold *** USER CONFIG ***
uint8_t release_trsh = 70;        // Release Threshold *** USER CONFIG ***

// Control Variables //
uint16_t pingSecondCounter;
uint16_t gpsSecondCounter;
uint16_t pingCounterTarget;
uint16_t gpsCounterTarget;

//#####################################################################################################//

//*** Functions ***//
void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 16384, resulting in 32.768kHz/16384 = 2Hz */
  | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
  pingSecondCounter = pingSecondCounter + 1;
  gpsSecondCounter = gpsSecondCounter + 1;
}

// Function 1 : Record and Store GPS data
void recGPS(){
  mTime = 0;
  digitalWrite(GPS_PIN, HIGH);
  Serial.println((unsigned int)gpsTimeout*1000);
  while (mTime <= (unsigned int)gpsTimeout*1000)
  {
    while (Serial1.available())
    {
      if (!gps.encode(Serial1.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println(F("Acquiring"));
        }else{
          Serial.println(gps.location.isUpdated());
          Serial.print(F("Location Age:"));
          Serial.println(gps.location.age());
          Serial.print(F("Time Age:"));
          Serial.println(gps.time.age());
          Serial.print(F("Date Age:"));
          Serial.println(gps.date.age());
          Serial.print(F("Satellites:"));
          Serial.println(gps.satellites.value());
          Serial.print(F("HDOP:"));
          Serial.println(gps.hdop.hdop());
        }       
      }      
    }
    if (gps.hdop.hdop() < (double)gpsHdop && gps.location.age() < 1000 && gps.time.age() < 1000 && mTime > 3000)
    {
      break;
    }  
  }   

  digitalWrite(GPS_PIN, LOW);
  data dat;

  if (gps.location.age() < 60000)
  {
    //pack data into struct
    lat = gps.location.lat();
    lng = gps.location.lng();
    dat.lat = gps.location.lat();
    dat.lng = gps.location.lng();
  }else{
    // pack data into struct with lat long = 0
    dat.lat = 0;
    dat.lng = 0;
  }
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    dat.datetime = (uint32_t)now();
    dat.locktime = mTime/1000;
    dat.hdop = gps.hdop.hdop();    
    
    Serial.println(dat.datetime);
    Serial.println(dat.lat);
    Serial.println(dat.lng);
    Serial.println(dat.locktime);
    Serial.println(dat.hdop);


  if (flash.powerUp())
  {
    Serial.println(F("Powered Up"));
    delay(500);
    Serial.println((int)sizeof(dat));
    wAdd = flash.getAddress(sizeof(dat));
    Serial.println(wAdd);
    if (flash.writeAnything(wAdd, dat))
    {
      Serial.println(F("Write Successful"));
      cnt = cnt + 1;
    }else
    {
      Serial.println(F("Write Failed"));
      Serial.println(flash.error(VERBOSE));
    }     
  }else
  {
    Serial.println(F("Power Up Failed"));
  }   
  flash.powerDown();

}

// Function 1-a : Secondary GPS Function used only in activation sequence
void acqGPS(){
  digitalWrite(GPS_PIN, HIGH);
      do{ 
        while (Serial1.available() > 0)
        {
          if (gps.encode(Serial1.read()))
          {
            if (!gps.location.isValid())
            {
              Serial.println(F("Not Valid"));
            }else{
              Serial.println(gps.location.isUpdated());
              Serial.print("Location Age:");
              Serial.println(gps.location.age());
              Serial.print("Time Age:");
              Serial.println(gps.time.age());
              Serial.print("Date Age:");
              Serial.println(gps.date.age());
              Serial.print("Satellites:");
              Serial.println(gps.satellites.value());
              Serial.print("HDOP:");
              Serial.println(gps.hdop.hdop());
            }
          }
        }
      }while(!gps.location.isValid());
    if (gps.location.age() < 60000)
    {
      //pack data into struct
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
    if (gps.time.isValid())
    {
      setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
      time_t n = now();
      strtTime = n;
      Serial.print(F("START TIME : ")); Serial.println(strtTime);
    }    
    digitalWrite(GPS_PIN, LOW);
}

// Function 2 : Read data from flash and send it through radio
void read_send(){ 
  data dat;

  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, dat))
    {
      // dat.id = tag;
      Serial.println(F("Read Successful"));
      Serial.println(dat.datetime);
      Serial.println(dat.hdop);
      Serial.println(dat.lat);
      Serial.println(dat.lng);
      Serial.println(dat.locktime);
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }
      LoRa.idle();
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&dat, sizeof(dat));
      LoRa.endPacket();
      LoRa.sleep();
}

// Function 3 : RTC Timer Interrupt Service Routine
void risr(){
  rtc_int_triggered = true;
  detachInterrupt(RINT);
  detachInterrupt(SINT);
}

// Function 4 : Check the elctrodes for sumbersion/surfacing events
void checkElectrodes(){
  MPR121.updateAll();
    Serial.println(F("Event"));
    for (int i = 0; i <= 12; i++) {
      if (MPR121.isNewTouch(i)) {
        Serial.print("E");
        Serial.print(i, DEC);
        Serial.println("+");  
        if (i == 1)
        {
          E1 = true;
        }
        if (i == 2)
        {
          E2 = true;
        }
        if (i == 3)
        {
          E3 = true;
        }
        if (i == 4)
        {
          E4 = true;
        }
        if (i == 5)
        {
          E5 = true;
        }   
        if (i == 0)
        {
          E6 = true;
        } 

      } else if (MPR121.isNewRelease(i)) {
        Serial.print("E");
        Serial.print(i, DEC);
        Serial.println("-");
        if (i == 1)
        {
          E1 = false;
        }
        if (i == 2)
        {
          E2 = false;
        }
        if (i == 3)
        {
          E3 = false;
        }
        if (i == 4)
        {
          E4 = false;
        }
        if (i == 5)
        {
          E5 = false;
        }
        if (i == 0)
        {
          E6 = false;
        }

      }
    }
    if (E1 == true && E2 == true && E3 == true && E4 == true && E5 == true && E6 == true)
    {
      surface = false;
    }
    if (E1 == false || E2 == false || E3 == false || E4 == false || E5 == false || E6 == false)
    {
      surface = true;
    }
    Serial.print(F("S:")); Serial.println(surface);
    delay(100);
}

// Function 5 : Pinger - Takes several inputs and sends ping
void Ping(float x, float y, uint16_t a, uint16_t c, byte d){

  ping p;

  p.devtyp = d;
  p.ta = a;
  p.la = x;
  p.ln = y;
  p.cnt = c;

  Serial.print(F("Size")); Serial.println((int)sizeof(p));
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&p, sizeof(p));
  LoRa.endPacket();
  LoRa.sleep();
  
}

// Function 6 : Look for and process inbound transmission right after ping.
void receive(unsigned int rcv_time){
  Serial.println(F("Receiving"));
  LoRa.idle();
  mTime = 0;
  int x = 0;
  do
  {  
    x = LoRa.parsePacket();
    if (x)
    {
      Serial.println(x);
    }
    
    
    if (x == 3)
    { 
      Serial.print(F("int : ")); Serial.println(x);
      struct request{
      uint16_t tag;
      byte request;
      }r;
      while (LoRa.available())
      {
        Serial.println(F("Reading in"));
        LoRa.readBytes((uint8_t*)&r, sizeof(r));
      }
      Serial.println(r.tag);
      Serial.println(r.request);
      if (r.tag == tag && r.request == (byte)82)
      {
        
        do
        {
          Serial.println("Init Stream");
          read_send();
          delay(50);
          rAdd = rAdd + 16;
          Serial.println(rAdd);
        
        } while (rAdd <= wAdd);

        delay(1000);
        struct resp{
        uint16_t tag;
        byte res;
        }r;
        r.res = (byte) 68;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
      }            
    }

    if (x == 21)
    {
      setttings set;

      while (LoRa.available())
      {
        Serial.println(F("Incoming Settings"));
        LoRa.readBytes((uint8_t*)&set, sizeof(set));
      }

      gpsFrequency = set.gpsFrq;
      gpsTimeout = set.gpsTout;
      gpsHdop = set.hdop;
      radioFrequency = set.radioFrq;
      rcv_duration = set.rcv_dur;
      
      Serial.println(set.gpsFrq);
      Serial.println(set.gpsTout);
      Serial.println(set.hdop);
      Serial.println(set.radioFrq);
      Serial.println(set.rcv_dur);
      delay(100);

      resPing r;
        r.resp = (byte)83;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
    }
  }while(mTime <= rcv_time);
  LoRa.sleep();
  delay(50);
}

// Function 7 : First activation ping - process active/sleep/wipe modes
void activationPing(){

  reqPing px1;
  resPing rs1;

  int x;

  px1.tag = tag;
  px1.request = (byte)73;


  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&px1, sizeof(px1));
  LoRa.endPacket();
  
  mTime = 0;
  while (mTime < 30000)
  {
    x = LoRa.parsePacket();
    if (x)
    {
      activation_resp_rcvd = true;
      Serial.println(F("Incoming"));
      Serial.println(x);
    }else{
      activation_resp_rcvd = false;
    }
    
    if (x == 3)
    {
      while (LoRa.available())
      {
        Serial.println(F("Message"));
        LoRa.readBytes((uint8_t*)&rs1, sizeof(rs1));
      }
      break;      
    }     
  } 
  LoRa.sleep();

  if (rs1.tag == tag && rs1.resp == (byte)70)
  {
    // Serial.print(F("System Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
    Serial.print(F("System Initialising with wipe"));
     /// Begin GPS and Acquire Lock ////
    acqGPS();
    
    wipe_memory = true;

    px1.request = (byte)106;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();
  }

  if (rs1.tag == tag && rs1.resp == (byte)71)
  {
    Serial.print(F("System Initialising without wipe"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
     /// Begin GPS and Acquire Lock ////
    // acqGPS();

    wipe_memory = false;

    px1.request = (byte)105;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();

  }

  if (rs1.tag == tag && rs1.resp == (byte)115){
        Serial.print(F("Indefinite Sleep"));
        EEPROM.put(1, false);
        Serial.println(EEPROM.read(1));
        delay(50);  
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();      
        sleep_cpu();
  }

  Serial.println(EEPROM.read(1));
  if (activation_resp_rcvd == false)
  {
    if (EEPROM.read(1) == false){
    Serial.println(F("SLEEP1"));
    delay(50);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    }else{
    Serial.println(F("Reset"));
    Serial.print(F("System Re - Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
     /// Begin GPS and Acquire Lock ////
    acqGPS();

    wipe_memory = false;

    px1.request = (byte)105;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();
    }
  }    
}

// Function 8 : MPR121 Interrupt Service Routine
void sisr(){
  mpr_int_triggered = true;
  detachInterrupt(RINT);
  detachInterrupt(SINT);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(RTC_PIN, OUTPUT);
  Wire.swapModule(&TWI1);
  Wire.usePullups();
  Wire.begin();
  RTC_init();

  //***********************************************//
  // if (!MPR121.begin(MPR121_ADDR)) {
  //   Serial.println(F("error setting up MPR121"));
  //   switch (MPR121.getError()) {
  //     case NO_ERROR:
  //       Serial.println(F("no error"));
  //       break;
  //     case ADDRESS_UNKNOWN:
  //       Serial.println(F("incorrect address"));
  //       break;
  //     case READBACK_FAIL:
  //       Serial.println(F("readback failure"));
  //       break;
  //     case OVERCURRENT_FLAG:
  //       Serial.println(F("overcurrent on REXT pin"));
  //       break;
  //     case OUT_OF_RANGE:
  //       Serial.println(F("electrode out of range"));
  //       break;
  //     case NOT_INITED:
  //       Serial.println(F("not initialised"));
  //       break;
  //     default:
  //       Serial.println(F("unknown error"));
  //       break;
  //   }
  //   while (1);
  // }
  // MPR121.setInterruptPin(MPR121_INT);
  // MPR121.setSamplePeriod(SAMPLE_INTERVAL_32MS);

  // MPR121.setTouchThreshold(touch_trsh);
  // MPR121.setReleaseThreshold(release_trsh);
  // MPR121.autoSetElectrodes();

  // if(!acce.begin()){
  //    Serial.println(F("Communication failed, check the connection and I2C address setting when using I2C communication."));
  //    delay(1000);
  // }else{
  // Serial.print(F("chip id : "));
  // Serial.println(acce.getID(),HEX);
  // }
  // acce.softReset();
  // acce.setRange(DFRobot_LIS2DW12::e4_g);
  // acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
  // acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  // acce.setWakeUpDur(/*dur = */2);
  // acce.setWakeUpThreshold(/*threshold = */0.3);
  // acce.setPowerMode(DFRobot_LIS2DW12::eContLowPwrLowNoise1_12bit);
  // acce.setActMode(DFRobot_LIS2DW12::eDetectAct);
  // acce.setInt1Event(DFRobot_LIS2DW12::eWakeUp);
  // acce.setDataRate(DFRobot_LIS2DW12::eRate_100hz);
  
  //***********************************************//
  LoRa.setPins(LCS, LRST, LDIO0);
  if(!LoRa.begin(867E6)){
    Serial.println(F("LoRa Failed Init"));
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(12);
  // LoRa.setSignalBandwidth(62.5E3);
  LoRa.sleep();
  //***********************************************//
  activationPing();
  //***********************************************//
  if(flash.powerUp()){
    Serial.println(F("Powered Up1"));
  }
  if(!flash.begin()){
    Serial.println(F("Flash again"));
    Serial.println(flash.error(VERBOSE));
  } 
  Serial.println(flash.getManID());
  if(flash.powerUp()){
    Serial.println(F("Powered Up"));
  }else{
    Serial.println(F("PWR UP Failed!"));
  }
  if (wipe_memory == true)
  {
    Serial.println(F("WIPING FLASH"));
    if(flash.eraseChip()){
    Serial.println(F("Memory Wiped"));  
    }else
    {
      Serial.println(flash.error(VERBOSE));
    }
  }else{
    rAdd = flash.getAddress(16); 
    wAdd = flash.getAddress(16);
  }    
  if(flash.powerDown()){
    Serial.println("Powered Down");
    digitalWrite(1, HIGH);
  }else{
    Serial.println(flash.error(VERBOSE));
  }

  // //***********************************************//
  
  // digitalWrite(RTC_PIN, HIGH);
  // rtc.set(strtTime);
  // Serial.println(rtc.get());
  // delay(100);
  // rtc.alarmPolarity(HIGH);
  // rtc.setAlarm(0, strtTime + 60*gpsFrequency);  
  // next_gps_wakeup = strtTime + 60*gpsFrequency;
  // Serial.println(next_gps_wakeup);
  // rtc.enableAlarm(0, ALM_MATCH_DATETIME);
  // rtc.enableAlarm(1, ALM_MATCH_DATETIME);
  // digitalWrite(RTC_PIN, LOW);
  // delay(100);
  // //***********************************************//
  // attachInterrupt(digitalPinToInterrupt(RINT), risr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SINT), sisr, CHANGE);
  //***********************************************//

  gpsCounterTarget = gpsFrequency*60;
  pingCounterTarget = radioFrequency*60;  
  gpsSecondCounter = gpsCounterTarget + 1;
  pingSecondCounter = pingCounterTarget + 1;

  //***********************************************//
  Serial.println("SYSTEM READY");
  Serial.flush();
  //***********************************************//
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (gpsSecondCounter >= gpsCounterTarget){
    Serial.println(F("AWAKE"));
  }
  //**************************************************************//
  if (mpr_int_triggered == true)
  {
    checkElectrodes();
    mpr_int_triggered = false;
  }
  if (surface)
  {
    if (gpsSecondCounter >= gpsCounterTarget)
    {
      recGPS();
      gpsSecondCounter = 0;
      if ((mTime/1000) < (gpsFrequency*60))
      {
        gpsCounterTarget = (gpsFrequency*60)-(mTime/1000);
      }else{
        gpsCounterTarget = gpsFrequency*60;
      }
    }
    if (pingSecondCounter >= pingCounterTarget)
    {
      Ping(lat, lng, tag, devType, cnt);
      receive(5000);
      pingSecondCounter = 0;
    }
  }
  checkElectrodes();
  Serial.flush();
  attachInterrupt(digitalPinToInterrupt(SINT), sisr, CHANGE);
  sleep_cpu();
}

//********************************************************************//
//************************* END OF CODE ******************************//
//********************************************************************//
