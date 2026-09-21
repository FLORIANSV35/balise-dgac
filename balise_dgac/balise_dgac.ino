// =============================================================
// BALISE DGAC — ESP32-C3
// Émet une balise DGAC (Remote ID, WiFi beacon canal 6) à partir
// du GPS d'un contrôleur de vol Betaflight lu en MSP.
// =============================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"

// =========================
// MSP DEFINITIONS (GELÉ)
// =========================
#define MSP_STATUS    101
#define MSP_RAW_GPS   106

#define MSP_BAUD      115200
#define MSP_RX_PIN    20
#define MSP_TX_PIN    21

HardwareSerial FC(0);

// =========================
// LED simple active LOW GPIO8 (GELÉ)
// =========================
#define LED_PIN 8

inline void setLed(bool on){
  digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// =========================
// DGAC HELPERS BIG-ENDIAN (GELÉ)
// =========================
inline void w16(uint8_t*b,size_t&p,uint16_t v){
  b[p++]=(v>>8)&0xFF; b[p++]=v&0xFF;
}
inline void w32(uint8_t*b,size_t&p,uint32_t v){
  b[p++]=(v>>24)&0xFF; b[p++]=(v>>16)&0xFF;
  b[p++]=(v>>8)&0xFF;  b[p++]=v&0xFF;
}
inline void wb(uint8_t*b,size_t&p,const void*s,size_t l){
  memcpy(&b[p],s,l); p+=l;
}
inline void tlv_u8(uint8_t*b,size_t&p,uint8_t t,uint8_t v){
  b[p++]=t; b[p++]=1; b[p++]=v;
}
inline void tlv_u16(uint8_t*b,size_t&p,uint8_t t,uint16_t v){
  b[p++]=t; b[p++]=2; w16(b,p,v);
}
inline void tlv_i16(uint8_t*b,size_t&p,uint8_t t,int16_t v){
  b[p++]=t; b[p++]=2;
  b[p++]=(v>>8)&0xFF; b[p++]=v&0xFF;
}
inline void tlv_i32(uint8_t*b,size_t&p,uint8_t t,int32_t v){
  b[p++]=t; b[p++]=4; w32(b,p,(uint32_t)v);
}
inline void tlv_bytes(uint8_t*b,size_t&p,uint8_t t,const void*s,uint8_t l){
  b[p++]=t; b[p++]=l; wb(b,p,s,l);
}

// =========================
// DGAC IDs (GELÉ)
// =========================
// À REMPLACER par ton propre identifiant DGAC (30 caractères exactement)
static const char ID_FR[31] = "000XXX000000000000000000000000";

const char    *BEACON_SSID = "RID-FR-BALISE";
uint8_t        mac_balise[6] = {0x02,0x11,0x22,0x33,0x44,0x55};

// =========================
// BEACON BUFFER (GELÉ)
// =========================
#define BEACON_MAX 256
uint8_t  beacon_frame[BEACON_MAX];
size_t   beacon_static_len = 0;
size_t   beacon_len = 0;

// =========================
// STATE VARIABLES (GELÉ)
// =========================
float    LAT_HOME_dyn = 0, LON_HOME_dyn = 0;
bool     home_set = false;

float    fc_lat = 0, fc_lon = 0;
int32_t  fc_alt = 0;
uint16_t fc_hdg = 0;
uint8_t  fc_speed = 0;
bool     fc_valid = false;

uint8_t  fc_num_sv = 0;
uint32_t last_fc_time = 0;

bool     armed = false, gps_ready = false;

// =========================
// DISTANCE (GELÉ) — sert au déclenchement d'émission tous les 30 m
// =========================
inline float dist_approx_m(float lat1,float lon1,float lat2,float lon2){
  const float K = 111320.0f;
  float dlat = (lat2-lat1)*K;
  float dlon = (lon2-lon1)*K*cosf(lat1*0.01745329252f);
  return sqrtf(dlat*dlat+dlon*dlon);
}

// =========================
// MSP (GELÉ)
// =========================
uint8_t msp_crc(uint8_t*b,uint8_t len){
  uint8_t c=0; for(uint8_t i=0;i<len;i++) c^=b[i]; return c;
}
bool msp_send(uint8_t cmd){
  uint8_t b[6]={'$','M','<',0,cmd,0};
  b[5]=msp_crc(&b[3],2); FC.write(b,6); return true;
}
bool msp_read(uint8_t*payload,uint8_t&len,uint8_t&cmd){
  static enum{s0,s1,s2,s3,s4,s5,s6}st=s0;
  static uint8_t cs=0,p=0;
  while(FC.available()){
    uint8_t c=FC.read();
    switch(st){
      case s0: if(c=='$') st=s1; break;
      case s1: st=(c=='M')?s2:s0; break;
      case s2: st=(c=='>')?s3:s0; break;
      case s3: len=c; cs=c; p=0; st=s4; break;
      case s4: cmd=c; cs^=c; st=len?s5:s6; break;
      case s5: payload[p++]=c; cs^=c; if(p>=len) st=s6; break;
      case s6: st=s0; return(cs==c);
    }
  }
  return false;
}
bool msp_get(uint8_t id,uint8_t*buf,uint8_t&sz){
  sz=0; msp_send(id);
  uint32_t t=millis(); uint8_t cmd=0;
  while(millis()-t<50){ if(msp_read(buf,sz,cmd)&&cmd==id) return true; }
  return false;
}
void readMSP(){
  uint8_t buf[64],sz=0;
  if(msp_get(MSP_STATUS,buf,sz)&&sz>=10){
    uint32_t flags=*(uint32_t*)&buf[6];
    bool na=flags&1;
    if(!home_set&&!armed&&na&&fc_valid){
      LAT_HOME_dyn=fc_lat; LON_HOME_dyn=fc_lon; home_set=true;
    }
    armed=na;
  }
  if(msp_get(MSP_RAW_GPS,buf,sz)&&sz>=16){
    fc_num_sv=buf[1];
    uint8_t fix_type=buf[0];
    if(!gps_ready&&fix_type>=1) gps_ready=true;
    if(fix_type>=1){
      int32_t la,lo; int16_t al; uint16_t sp,cr;
      memcpy(&la,&buf[2],4); memcpy(&lo,&buf[6],4);
      memcpy(&al,&buf[10],2); memcpy(&sp,&buf[12],2); memcpy(&cr,&buf[14],2);
      fc_lat=la/1e7f; fc_lon=lo/1e7f;
      fc_alt=al; fc_speed=sp/100; fc_hdg=cr/10;
      fc_valid=true; last_fc_time=millis();
    }
  }
  if(millis()-last_fc_time>2000) fc_valid=false;
}

// =========================
// DGAC PAYLOAD (GELÉ)
// =========================
void build_dgac_tlv_payload(uint8_t*b,size_t&p){
  int32_t la=0,lo=0;
  if(fc_valid){ la=(int32_t)(fc_lat*1e5); lo=(int32_t)(fc_lon*1e5); }
  else if(home_set){ la=(int32_t)(LAT_HOME_dyn*1e5); lo=(int32_t)(LON_HOME_dyn*1e5); }
  tlv_u8   (b,p,0x01,0x01);
  tlv_bytes(b,p,0x02,ID_FR,30);
  tlv_i32  (b,p,0x04,la);
  tlv_i32  (b,p,0x05,lo);
  tlv_i16  (b,p,0x06,(int16_t)(fc_valid?fc_alt:120));
  tlv_i32  (b,p,0x08,(int32_t)(LAT_HOME_dyn*1e5));
  tlv_i32  (b,p,0x09,(int32_t)(LON_HOME_dyn*1e5));
  tlv_u8   (b,p,0x0A,fc_valid?fc_speed:0);
  tlv_u16  (b,p,0x0B,fc_valid?fc_hdg:0);
}
void build_beacon_static(){
  size_t p=0; uint8_t*b=beacon_frame;
  b[p++]=0x80; b[p++]=0x00; b[p++]=0x00; b[p++]=0x00;
  memset(&b[p],0xFF,6); p+=6;
  wb(b,p,mac_balise,6); wb(b,p,mac_balise,6);
  b[p++]=0; b[p++]=0;
  memset(&b[p],0,8); p+=8;
  b[p++]=0x64; b[p++]=0x00; b[p++]=0x11; b[p++]=0x04;
  b[p++]=0x00; uint8_t sl=strlen(BEACON_SSID); b[p++]=sl;
  wb(b,p,BEACON_SSID,sl);
  b[p++]=0x03; b[p++]=0x01; b[p++]=6;
  beacon_static_len=p;
}
void update_beacon_dynamic(){
  size_t p=beacon_static_len; uint8_t*b=beacon_frame;
  if(beacon_static_len+90>BEACON_MAX){ beacon_len=0; return; }
  size_t ie_start=p;
  b[p++]=0xDD; size_t len_pos=p++;
  b[p++]=0x6A; b[p++]=0x5C; b[p++]=0x35; b[p++]=0x01;
  build_dgac_tlv_payload(b,p);
  b[len_pos]=p-ie_start-2;
  beacon_len=p;
}

// =========================
// MAIN
// =========================
uint32_t last_emit_time = 0;
float    last_emit_lat  = 0, last_emit_lon = 0;
bool     first_emit     = true;

void setup(){
  Serial.begin(115200);
  delay(300);

  FC.begin(MSP_BAUD, SERIAL_8N1, MSP_RX_PIN, MSP_TX_PIN);

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(78); // 19.5 dBm = maximum exposé par l'API (unités de 0.25dBm)
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

  build_beacon_static();

  Serial.println("[BALISE] prete");
}

void loop(){
  readMSP();

  if(!gps_ready){
    // Pas de GPS : beacon inactive, LED éteinte
    setLed(false);
    delay(5);
    return;
  }

  uint32_t now = millis();
  bool emit = false;

  if(first_emit){ emit=true; first_emit=false; }
  if(now-last_emit_time>=3000) emit=true;
  if(fc_valid&&dist_approx_m(last_emit_lat,last_emit_lon,fc_lat,fc_lon)>30.0f) emit=true;

  if(emit){
    update_beacon_dynamic();
    if(beacon_len>0){
      for(int i=0;i<3;i++){
        esp_wifi_80211_tx(WIFI_IF_STA,beacon_frame,beacon_len,false);
        delayMicroseconds(5000);
      }
    }
    last_emit_time=now;
    last_emit_lat=fc_lat;
    last_emit_lon=fc_lon;
  }

  setLed(true); // LED fixe : la balise émet
  delay(5);
}
