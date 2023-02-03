////////////////////////////////////////////
/////      Firmware Definitions        /////
////////////////////////////////////////////

///// Device Definitions /////

// Device ID //

const uint16_t tag = 10109;
const uint8_t devType = 107;

// Firmware Version //
const float firmwareVersion = 2.0;

//// Pin Definitions ////

#define GPS_PIN PIN_PC3
#define RTC_PIN PIN_PD7
#define RINT PIN_PD6
#define AINT1 PIN_PD4
#define AINT2 PIN_PD5
#define LCS PIN_PA7
#define FCS PIN_PC2
#define LRST PIN_PA3
#define LDIO0 PIN_PF5
#define LDIO1 PIN_PF4


//// Structs ////

//// Request Ping ////

struct reqPing{
    uint16_t tag;
    byte request;
  };

struct resPing{
    uint16_t tag;
    byte resp;
  };

struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    byte hdop;
  };

struct ping{
    uint16_t ta;    
    uint16_t cnt;
    float la;
    float ln;
    uint8_t devtyp;
    bool mortality;
  }; 

struct setttings{
    uint32_t pingTime;
    uint16_t act_trsh;
    uint16_t act_gps_frq;
    uint16_t act_duration;
    uint16_t gpsFrq;
    uint16_t gpsTout;
    uint8_t hdop;
    uint8_t radioFrq;
    uint8_t rcv_dur;
    uint8_t sch_dur;
    uint8_t sch_rpt_dur;
    bool act_enabled;
    bool sch_enabled;
  } __attribute__((__packed__));



