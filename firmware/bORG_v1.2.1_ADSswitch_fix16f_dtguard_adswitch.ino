// SPDX-License-Identifier: GPL-3.0-or-later
/*******************************************************
 * bORG v1.2.1 (I2C toggle build) – Korg Modwave MKI -> Arduino Pro Micro USB-MIDI
 * Target: Pro Micro 3.3 V
 *
 * === Build-time switch ===
 *   #define USE_I2C 0   -> NO I2C (no MCP/ADS). FN on D15. Scanner+SysEx only.
 *   #define USE_I2C 1   -> WITH I2C (MCP23017 @0x20, ADS1115 @0x48), as designed.
 *
 * I²C wiring (when USE_I2C=1):
 *   SDA=D2, SCL=D3. MCP23017 @0x20: GPA0..4 inputs (sustain/extra/oct+/oct-/FN), GPB0..5 RGB LEDs.
 *   ADS1115 @0x48: A0=Joy X (Pitch Bend), A1=Joy Y (ModWheel).
 *
 * SysEx (0x7D non-commercial):
 *  v1.0: F0 7D 01 cc F7 (curve), 02 (cal start), 03 (cal stop), 04 (save), 05 (reset), 06 vv (fixedVel), 0A (status)
 *  v1.1: F0 7D 07 cc F7 (MIDI ch 1..16), 08 pp (Program Change 0..127)
 *  FN+keys: channel (16), curve (8), C4 start, D4 stop, E4 save, F4 reset, G4 status
 *
 * License: GPL-3.0-or-later. KORG és KORG Modwave védjegyek a KORG Inc. tulajdonai. No warranty.
 *******************************************************/

#define USE_ADS 0  // 0: joystick/ADS disabled; 1: enable ADS1115
#define USE_I2C 1          // <<<<<<  set to 1 when MCP/ADS wired OK
#define IGNORE_FN_FOR_TEST 0
#define SHORT_FOCUS_WAIT_US 6000
#define STRICT_PAIRING 1
#define ARM_TIMEOUT_TICKS 20000  // ~10 ms @ 0.5 µs tick

#include <Arduino.h>
#include <MIDIUSB.h>
#include <EEPROM.h>
#include <math.h>

// --- DT guard helpers (order- and wrap-safe) ---
#ifndef MAX_DT_TICKS
  #define MAX_DT_TICKS 40000  // ~20 ms window @ 0.5 µs tick
#endif
#ifndef MIN_DT_TICKS
  #define MIN_DT_TICKS 300    // ~150 µs threshold
#endif
static inline uint16_t dt_minwrap(uint16_t a, uint16_t b){
  uint16_t d1=(uint16_t)(b-a);
  uint16_t d2=(uint16_t)(a-b);
  return (d1<d2)?d1:d2;
}

// === Pairing debug (optional) ===
//#define DEBUG_PAIR 0  // set to 1 to enable KS/KF index-time logs
inline void dbgKS(uint8_t idx){
#if DEBUG_PAIR
  Serial.print("KS@"); Serial.print(idx); Serial.print(" t="); Serial.println(tnow());
#endif
}
inline void dbgKF(uint8_t idx){
#if DEBUG_PAIR
  Serial.print("KF@"); Serial.print(idx); Serial.print(" t="); Serial.println(tnow());
#endif
}
#if USE_I2C
  #include <Wire.h>
  #include <Adafruit_MCP23X17.h>
  #include <Adafruit_ADS1X15.h>
  Adafruit_MCP23X17 mcp;
  Adafruit_ADS1115  ads;
  bool g_ads_ok=false;
#endif

/*** Pins ***/
const uint8_t KS_PINS[4] = {14,5,7,9};     // KS0..KS3 (LOW=active)
const uint8_t KF_PINS[4] = {A0,A1,A2,A3};  // KF0..KF3 (LOW=active)
const uint8_t KBD0=4, KBD1=6, KBD2=8, KBD3=10, KBD4=16;
const uint8_t LED=17;
#if USE_I2C
  const uint8_t MCP_SUSTAIN=0, MCP_EXTRA=1, MCP_OCT_UP=2, MCP_OCT_DN=3, MCP_FN=4;
  const uint8_t MCP_DOWN_R=8, MCP_DOWN_G=9, MCP_DOWN_B=10, MCP_UP_R=11, MCP_UP_G=12, MCP_UP_B=13;
#else
  const uint8_t PIN_FN=15; // local FN input when no I2C
#endif

/*** Scan/debounce ***/
const uint8_t  SAMPLES=1, MAJ=1;
const uint16_t SAMPLE_DELAY_US=0;
const uint16_t COL_SETTLE_US=150;
/*** NoteOff ***/
const uint8_t  RELEASE_DEBOUNCE_MS=3;
const uint8_t  MIN_NOTE_LEN_MS=8;
const bool     USE_NOTE_OFF_VELOCITY=true;
/*** MIDI ***/
uint8_t MIDI_CH=1;
const uint8_t MIN_VEL=1, MAX_VEL=127;
uint8_t NOTE_BASE=48;
int8_t  octaveShift=0;
/*** Curves ***/
enum Curve:uint8_t{CURVE_LINEAR, CURVE_FLAT, CURVE_STEEP, CURVE_PIANO, CURVE_SYNTH, CURVE_ORGAN_FIXED, CURVE_SOFT, CURVE_HARD};
float   dt_fast_us=600.0f, dt_slow_us=20000.0f;
Curve   currentCurve=CURVE_LINEAR;
uint8_t fixedVelocity=100;
bool    calib_locked=true;
uint16_t learn_count=0;
/*** Note map ***/
struct Contact{uint8_t panel,col,row;};
const Contact NOTE_KS[37]={{1,1,3},{1,1,2},{1,1,1},{1,1,0},{1,0,3},{1,0,2},{1,0,1},{1,0,0},{0,7,3},{0,7,2},{0,7,1},{0,7,0},{0,6,0},{0,5,3},{0,5,2},{0,5,1},{0,5,0},{0,4,3},{0,4,2},{0,4,1},{0,4,0},{0,3,3},{0,3,2},{0,3,1},{0,3,0},{0,2,3},{0,2,2},{0,2,1},{0,2,0},{0,1,3},{0,1,2},{0,1,1},{0,1,0},{0,0,3},{0,0,2},{0,0,1},{0,0,0}};
const Contact NOTE_KF[37]={{1,1,3},{1,1,2},{1,1,1},{1,1,0},{1,0,3},{1,0,2},{1,0,1},{1,0,0},{0,7,3},{0,7,2},{0,7,1},{0,7,0},{0,6,0},{0,5,3},{0,5,2},{0,5,1},{0,5,0},{0,4,3},{0,4,2},{0,4,1},{0,4,0},{0,3,3},{0,3,2},{0,3,1},{0,3,0},{0,2,3},{0,2,2},{0,2,1},{0,2,0},{0,1,3},{0,1,2},{0,1,1},{0,1,0},{0,0,3},{0,0,2},{0,0,1},{0,0,0}};
int8_t REV_KS[2][8][4], REV_KF[2][8][4];
/*** State ***/
bool lastKS[2][8][4]={{{0}}}, lastKF[2][8][4]={{{0}}};
uint16_t tKS_ticks[37]={0}, tKF_ticks[37]={0};
bool noteOnSent[37]={0};
uint32_t noteOnMs[37]={0};
bool releaseArm[37]={0};

#if STRICT_PAIRING
static bool armedKS[37]={0};
static bool armedKF[37]={0};
#endif
uint16_t tFirstReleaseTicks[37]={0};
/*** Timer1 ***/
static inline void timer1_init(){ TCCR1A=0; TCCR1B=_BV(CS11); TCNT1=0; }
static inline uint16_t tnow(){ return TCNT1; }
/*** EEPROM ***/
struct Persist{uint32_t magic; float dt_fast,dt_slow; uint8_t curve,locked,fixedVel,_pad[5];};
const uint32_t MAGIC=0x4B4D5744UL; const int EEPROM_ADDR=0;
void saveEEP(){ Persist p{MAGIC,dt_fast_us,dt_slow_us,(uint8_t)currentCurve,(uint8_t)(calib_locked?1:0),fixedVelocity,{0}}; EEPROM.put(EEPROM_ADDR,p); }
bool loadEEP(){ Persist p; EEPROM.get(EEPROM_ADDR,p); if(p.magic!=MAGIC) return false; dt_fast_us=p.dt_fast; dt_slow_us=p.dt_slow; currentCurve=(Curve)p.curve; calib_locked=p.locked!=0; fixedVelocity=p.fixedVel? p.fixedVel:100; return true; }
void resetEEP(){ Persist p{}; EEPROM.put(EEPROM_ADDR,p); }
/*** HW helpers ***/
static inline void setEnableForPanel(uint8_t panel){ digitalWrite(KBD3, panel ? HIGH : LOW); digitalWrite(KBD4, LOW); }
static inline void selCol(uint8_t y){ digitalWrite(KBD0,(y&1)?HIGH:LOW); digitalWrite(KBD1,(y&2)?HIGH:LOW); digitalWrite(KBD2,(y&4)?HIGH:LOW); delayMicroseconds(COL_SETTLE_US); }
static inline bool rd(uint8_t pin){ uint8_t hit=0; for(uint8_t i=0;i<SAMPLES;i++){ if(digitalRead(pin)==LOW) hit++; if(SAMPLE_DELAY_US) delayMicroseconds(SAMPLE_DELAY_US);} return hit>=MAJ; }
/*** MIDI ***/
static inline void midiOn(uint8_t note,uint8_t vel){ midiEventPacket_t p={0x09,(uint8_t)(0x90|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p); }
static inline void midiOffV(uint8_t note,uint8_t vel){ midiEventPacket_t p={0x08,(uint8_t)(0x80|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p); }
static inline void sendCC(uint8_t cc,uint8_t val){ midiEventPacket_t p={0x0B,(uint8_t)(0xB0|(MIDI_CH-1)),cc,val}; MidiUSB.sendMIDI(p); }
static inline void sendPB(int16_t bend){ uint16_t u=(uint16_t)(bend+8192); uint8_t lsb=u&0x7F, msb=(u>>7)&0x7F; midiEventPacket_t p={0x0E,(uint8_t)(0xE0|(MIDI_CH-1)),lsb,msb}; MidiUSB.sendMIDI(p); }
/*** Curves ***/
static inline float curve_piano(float x){ const float k=6.0f; return 1.0f/(1.0f+expf(-k*(0.5f-x))); }
static inline float applyCurve01(float x, uint8_t c){
  if(x<0) x=0; if(x>1) x=1;
  switch(c){
    case CURVE_LINEAR: return 1.0f-x; case CURVE_FLAT: return 1.0f-powf(x,1.6f); case CURVE_STEEP: return 1.0f-powf(x,0.7f);
    case CURVE_PIANO: return curve_piano(x); case CURVE_SYNTH: return 1.0f-(1.0f-expf(-3.0f*x))/(1.0f-expf(-3.0f));
    case CURVE_ORGAN_FIXED: return -1.0f; case CURVE_SOFT: return 1.0f-powf(x,2.2f); case CURVE_HARD: return 1.0f-powf(x,0.5f);
    default: return 1.0f-x;
  }
}
static inline void updateCalib(uint32_t dt_us){
  if(calib_locked) return; const float AF=0.25f, AS=0.18f, D=0.02f;
  float eF=(float)dt_us-dt_fast_us; dt_fast_us += (eF<0 ? AF*eF : D*eF); if(dt_fast_us<150.0f) dt_fast_us=150.0f;
  float eS=(float)dt_us-dt_slow_us; dt_slow_us += (eS>0 ? AS*eS : D*eS); if(dt_slow_us>40000.0f) dt_slow_us=40000.0f;
  if(dt_slow_us-dt_fast_us<3000.0f) dt_slow_us=dt_fast_us+3000.0f;
}
static inline uint8_t vel_from_dt_ticks(uint16_t dt_ticks){
  if(currentCurve==CURVE_ORGAN_FIXED){ uint8_t v=fixedVelocity; if(v<MIN_VEL)v=MIN_VEL; if(v>MAX_VEL)v=MAX_VEL; return v; }
  uint32_t dt_us=(uint32_t)dt_ticks>>1;
  float x=(float)((int32_t)dt_us-(int32_t)dt_fast_us)/(float)(dt_slow_us-dt_fast_us); if(x<0)x=0; if(x>1)x=1;
  float y=applyCurve01(x, (uint8_t)currentCurve); if(y<0)y=0; if(y>1)y=1;
  uint8_t v=(uint8_t)lroundf(y*(MAX_VEL-MIN_VEL)+MIN_VEL); if(v<MIN_VEL)v=MIN_VEL; if(v>MAX_VEL)v=MAX_VEL; return v;
}
/*** Focus helper ***/
static inline bool shortFocusWait(uint8_t panel,uint8_t col,uint8_t row,bool wantKS,uint16_t &tOutTicks){
  setEnableForPanel(panel); selCol(col); uint32_t us=0;
  while(us<SHORT_FOCUS_WAIT_US){ bool v=rd(wantKS?KS_PINS[row]:KF_PINS[row]); if(v){ tOutTicks=tnow(); return true; } delayMicroseconds(3); us+=3; }
  return false;
}
/*** Rev map ***/
void buildRev(){
  for(uint8_t p=0;p<2;p++) for(uint8_t y=0;y<8;y++) for(uint8_t r=0;r<4;r++){ REV_KS[p][y][r]=-1; REV_KF[p][y][r]=-1; }
  for(uint8_t i=0;i<37;i++){ REV_KS[NOTE_KS[i].panel][NOTE_KS[i].col][NOTE_KS[i].row]=i; REV_KF[NOTE_KF[i].panel][NOTE_KF[i].col][NOTE_KF[i].row]=i; }
}
/*** Status ***/
bool g_eep_loaded=false;
const char* curveName(uint8_t c){ switch(c){ case CURVE_LINEAR:return "LINEAR"; case CURVE_FLAT:return "FLAT"; case CURVE_STEEP:return "STEEP"; case CURVE_PIANO:return "PIANO"; case CURVE_SYNTH:return "SYNTH"; case CURVE_ORGAN_FIXED:return "ORGAN_FIXED"; case CURVE_SOFT:return "SOFT"; case CURVE_HARD:return "HARD"; default:return "UNKNOWN"; } }
void printStatus(){
  Serial.println(F("=== bORG v1.2.1 (I2C toggle) STATUS ==="));
  Serial.print(F("EEPROM loaded: ")); Serial.println(g_eep_loaded?F("YES"):F("NO"));
  Serial.print(F("Calib locked: ")); Serial.println(calib_locked?F("YES"):F("NO"));
  Serial.print(F("dt_fast_us: ")); Serial.println(dt_fast_us,1);
  Serial.print(F("dt_slow_us: ")); Serial.println(dt_slow_us,1);
  Serial.print(F("Curve: ")); Serial.println(curveName((uint8_t)currentCurve));
  Serial.print(F("Fixed velocity: ")); Serial.println(fixedVelocity);
  Serial.print(F("NOTE_BASE: ")); Serial.println(NOTE_BASE);
  Serial.print(F("MIDI_CH: ")); Serial.println(MIDI_CH);
  Serial.print(F("Octave shift: ")); Serial.println(octaveShift);
  Serial.println(F("==============================="));
}
/*** FN detect ***/
#if USE_I2C
  inline bool mcpRead(uint8_t pin){ return mcp.digitalRead(pin)==LOW; }
  inline bool fnActive(){ return IGNORE_FN_FOR_TEST ? false : mcpRead(MCP_FN); }
#else
  inline bool fnActive(){ return IGNORE_FN_FOR_TEST ? false : (digitalRead(PIN_FN)==LOW); }
#endif
/*** Joystick & buttons (only when I2C on) ***/
void processJoystick(){
#if (USE_I2C && USE_ADS)

#if USE_I2C
  static uint32_t last=0; if(millis()-last<5) return; last=millis();
  if(!g_ads_ok){ sendPB(0); sendCC(1,0); return; }
  int16_t vx=ads.readADC_SingleEnded(0), vy=ads.readADC_SingleEnded(1);
  const int16_t MID=16384, DZ=800, SPAN=16384;
  int32_t d=(int32_t)vx-MID; if(abs(d)<=DZ) d=0; else if(d>0) d-=DZ; else d+=DZ;
  long pb=(long)d*8191/(SPAN-DZ); if(pb>8191) pb=8191; if(pb<-8191) pb=-8191; sendPB((int16_t)pb);
  uint8_t cc=(uint8_t)constrain((long)vy*127/32767,0,127); sendCC(1,cc);
#endif

#else
  return;
#endif
}
void handleButtons(){
#if USE_I2C
  static uint32_t last=0; if(millis()-last<10) return; last=millis();
  static bool sustPrev=false; bool sustNow=mcpRead(MCP_SUSTAIN); if(sustNow!=sustPrev){ sendCC(64, sustNow?127:0); sustPrev=sustNow; }
  static bool extraPrev=false; bool extraNow=mcpRead(MCP_EXTRA); if(extraNow!=extraPrev){ sendCC(67, extraNow?127:0); extraPrev=extraNow; }
  static bool upPrev=false, dnPrev=false; bool upNow=mcpRead(MCP_OCT_UP), dnNow=mcpRead(MCP_OCT_DN);
  if(upNow && !upPrev){ if(octaveShift<3){ octaveShift++; } } if(dnNow && !dnPrev){ if(octaveShift>-3){ octaveShift--; } }
  upPrev=upNow; dnPrev=dnNow;
#endif
}
void showOctaveLED(){
#if USE_I2C
  auto setRGB=[&](bool up,uint8_t r,uint8_t g,uint8_t b){
    if(up){ mcp.digitalWrite(MCP_UP_R,r?HIGH:LOW); mcp.digitalWrite(MCP_UP_G,g?HIGH:LOW); mcp.digitalWrite(MCP_UP_B,b?HIGH:LOW); }
    else  { mcp.digitalWrite(MCP_DOWN_R,r?HIGH:LOW); mcp.digitalWrite(MCP_DOWN_G,g?HIGH:LOW); mcp.digitalWrite(MCP_DOWN_B,b?HIGH:LOW); }
  };
  setRGB(false,0,0,0); setRGB(true,0,0,0); if(octaveShift==0) return;
  bool up=octaveShift>0; uint8_t mag=abs(octaveShift); uint8_t r=0,g=0; if(mag==1) g=1; else if(mag==2){ r=1; g=1; } else r=1;
  setRGB(up,r,g,0);
#endif
}
/*** FN shortcuts ***/
void handleFnShortcutByNote(uint8_t midiNote){
  if(midiNote>=NOTE_BASE && midiNote<=NOTE_BASE+15){ MIDI_CH=1+(midiNote-NOTE_BASE); return; }
  if(midiNote>=NOTE_BASE+16 && midiNote<=NOTE_BASE+23){ uint8_t cc=midiNote-(NOTE_BASE+16); if(cc<=7) currentCurve=(Curve)cc; return; }
  switch(midiNote){ case 60: calib_locked=false; learn_count=0; break; case 62: calib_locked=true; break; case 64: saveEEP(); break;
    case 65: resetEEP(); dt_fast_us=600.0f; dt_slow_us=20000.0f; currentCurve=CURVE_LINEAR; calib_locked=true; fixedVelocity=100; break;
    case 67: printStatus(); break; default: break; }
}
/*** SysEx ***/
void handleSysExBuffer(uint8_t *buf,uint8_t len){
  if(len<4||buf[0]!=0xF0||buf[len-1]!=0xF7||buf[1]!=0x7D) return;
  uint8_t cmd=buf[2];
  switch(cmd){
    case 0x01: if(len>=5){ uint8_t c=buf[3]; if(c<=7) currentCurve=(Curve)c; } break;
    case 0x02: calib_locked=false; learn_count=0; break;
    case 0x03: calib_locked=true; break;
    case 0x04: saveEEP(); break;
    case 0x05: resetEEP(); dt_fast_us=600.0f; dt_slow_us=20000.0f; currentCurve=CURVE_LINEAR; calib_locked=true; fixedVelocity=100; break;
    case 0x06: if(len>=5){ uint8_t v=buf[3]; v=constrain(v,1,127); fixedVelocity=v; } break;
    case 0x07: if(len>=5){ uint8_t ch=buf[3]; if(ch>=1&&ch<=16) MIDI_CH=ch; } break;
    case 0x08: if(len>=5){ uint8_t pp=buf[3]; midiEventPacket_t p={0x0C,(uint8_t)(0xC0|(MIDI_CH-1)),pp,0}; MidiUSB.sendMIDI(p);} break;
    case 0x0A: printStatus(); break;
    default: break;
  }
}
void pollMIDIin(){
  static uint8_t sx[64]; static uint8_t sxLen=0; midiEventPacket_t rx;
  while((rx=MidiUSB.read()).header!=0){
    uint8_t cin=rx.header & 0x0F;
    if(cin==0x04||cin==0x07||cin==0x06||cin==0x05){
      uint8_t b[3]={rx.byte1,rx.byte2,rx.byte3};
      for(uint8_t i=0;i<3;i++){
        if(b[i]==0 && (cin==0x06||cin==0x05)) break;
        if(sxLen<sizeof(sx)) sx[sxLen++]=b[i];
        if(b[i]==0xF7){ handleSysExBuffer(sx,sxLen); sxLen=0; break; }
      }
    }
  }
}
/*** Timer ***/
static inline void heartbeat(){ static uint32_t t=0; if(millis()-t>=500){ t=millis(); digitalWrite(LED,!digitalRead(LED)); } }
/*** Setup ***/
void buildRev();
void setup(){
  pinMode(LED,OUTPUT); digitalWrite(LED,LOW);
  pinMode(KBD0,OUTPUT); pinMode(KBD1,OUTPUT); pinMode(KBD2,OUTPUT);
  pinMode(KBD3,OUTPUT); pinMode(KBD4,OUTPUT);
  for(uint8_t r=0;r<4;r++){ pinMode(KS_PINS[r],INPUT); pinMode(KF_PINS[r],INPUT); }
#if !USE_I2C
  pinMode(PIN_FN,INPUT_PULLUP);
#endif
  digitalWrite(KBD3,LOW); digitalWrite(KBD4,LOW);
  digitalWrite(KBD0,LOW); digitalWrite(KBD1,LOW); digitalWrite(KBD2,LOW);
  timer1_init(); buildRev(); Serial.begin(115200);
  if(loadEEP()) g_eep_loaded=true; else { g_eep_loaded=false; dt_fast_us=600.0f; dt_slow_us=20000.0f; currentCurve=CURVE_LINEAR; calib_locked=true; fixedVelocity=100; }
#if USE_I2C
  Wire.begin();
  if(mcp.begin_I2C(0x20)){
    for(uint8_t i=0;i<=4;i++) mcp.pinMode(i, INPUT_PULLUP);
    for(uint8_t i=8;i<=13;i++){ mcp.pinMode(i, OUTPUT); mcp.digitalWrite(i,LOW); }
  }
#if (USE_I2C && USE_ADS)
  if(ads.begin(0x48)){ ads.setGain(GAIN_ONE); g_ads_ok=true; }
#endif

#endif
}
/*** Velocity trigger ***/
static inline bool tryComputeVelAndTrigger(uint8_t idx,uint16_t tA,uint16_t tB){
  if(tA==0 || tB==0) return false;
  uint16_t dt = dt_minwrap(tA, tB);
  if(dt < MIN_DT_TICKS) return false;
  if(dt > MAX_DT_TICKS) return false;
  uint8_t vel = vel_from_dt_ticks(dt);
  if(fnActive()){
    handleFnShortcutByNote((uint8_t)(NOTE_BASE + idx + 12*octaveShift));
  }else{
    midiOn((uint8_t)(NOTE_BASE + idx + 12*octaveShift), vel);
    noteOnSent[idx] = true;
    noteOnMs[idx] = millis();
    releaseArm[idx] = false;
    tFirstReleaseTicks[idx] = 0;
    updateCalib((uint32_t)dt >> 1);
  }
  return true;
}
else{
    midiOn((uint8_t)(NOTE_BASE+idx+12*octaveShift), vel);
    noteOnSent[idx]=true; noteOnMs[idx]=millis(); releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; updateCalib((uint32_t)dt>>1);
  }
  return true;
}
/*** Loop ***/
void loop(){
  heartbeat();
  pollMIDIin();
  handleButtons();
  processJoystick();

  bool any=false;
  for(uint8_t panel=0; panel<2; panel++){
    setEnableForPanel(panel);
    for(uint8_t y=0; y<8; y++){
      selCol(y);
      for(uint8_t r=0; r<4; r++){
        const bool vKS=rd(KS_PINS[r]), vKF=rd(KF_PINS[r]);
        const int8_t iKS=REV_KS[panel][y][r], iKF=REV_KF[panel][y][r];

        if(iKS>=0){
          if(vKS && !lastKS[panel][y][r]){
            uint16_t tNow=tnow(); tKS_ticks[iKS]=tNow; dbgKS(iKS);
#if STRICT_PAIRING
  armedKS[iKS]=true;
  if(armedKF[iKS]){ armedKF[iKS]=false; armedKS[iKS]=false; if(!noteOnSent[iKS]){ if(tryComputeVelAndTrigger(iKS,tKS_ticks[iKS],tKF_ticks[iKS])) any=true; } tKS_ticks[iKS]=0; tKF_ticks[iKS]=0; }
#endif
            if(!noteOnSent[iKS] && tKF_ticks[iKS]==0){ uint16_t t2; if(shortFocusWait(panel,y,r,false,t2)) tKF_ticks[iKS]=t2; }
            if(!noteOnSent[iKS]){ if(tryComputeVelAndTrigger(iKS,tKS_ticks[iKS],tKF_ticks[iKS])) any=true; }
          }
          lastKS[panel][y][r]=vKS;
        }
        if(iKF>=0){
          if(vKF && !lastKF[panel][y][r]){
            uint16_t tNow=tnow(); tKF_ticks[iKF]=tNow; dbgKF(iKF);
#if STRICT_PAIRING
  armedKF[iKF]=true;
  if(armedKS[iKF]){ armedKF[iKF]=false; armedKS[iKF]=false; if(!noteOnSent[iKF]){ if(tryComputeVelAndTrigger(iKF,tKS_ticks[iKF],tKF_ticks[iKF])) any=true; } tKS_ticks[iKF]=0; tKF_ticks[iKF]=0; }
#endif
            if(!noteOnSent[iKF] && tKS_ticks[iKF]==0){ uint16_t t2; if(shortFocusWait(panel,y,r,true,t2)) tKS_ticks[iKF]=t2; }
            if(!noteOnSent[iKF]){ if(tryComputeVelAndTrigger(iKF,tKS_ticks[iKF],tKF_ticks[iKF])) any=true; }
          }
          lastKF[panel][y][r]=vKF;
        }

        auto processOffFor=[&](int8_t idx,bool vks,bool vkf){
          if(idx<0) return;
          if(!noteOnSent[idx]){ releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; return; }
          uint32_t nowMs=millis(); if(nowMs-noteOnMs[idx]<MIN_NOTE_LEN_MS) return;
          const bool bothOpenNow=(!vks && !vkf);
          if(!releaseArm[idx] && (!vks || !vkf)){ releaseArm[idx]=true; tFirstReleaseTicks[idx]=tnow(); }
          static uint32_t bothOpenSinceMs[37]={0};
          if(bothOpenNow){
            if(bothOpenSinceMs[idx]==0) bothOpenSinceMs[idx]=nowMs;
            if(nowMs-bothOpenSinceMs[idx]>=RELEASE_DEBOUNCE_MS){
              uint8_t offVel=0;
              if(USE_NOTE_OFF_VELOCITY && releaseArm[idx]){
                uint16_t t2=tnow(); int16_t d=(int16_t)(t2-tFirstReleaseTicks[idx]); if(d<0) d=-d;
                offVel=vel_from_dt_ticks((uint16_t)d);
              }
              midiOffV((uint8_t)(NOTE_BASE+idx+12*octaveShift), offVel);
              noteOnSent[idx]=false; tKS_ticks[idx]=0; tKF_ticks[idx]=0; releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; bothOpenSinceMs[idx]=0; any=true;
            }
          }else bothOpenSinceMs[idx]=0;
        };
        processOffFor(iKS,vKS,vKF);
        processOffFor(iKF,vKS,vKF);
      }
    }
  }
#if STRICT_PAIRING
  for(uint8_t ii=0; ii<37; ++ii){
    if(armedKS[ii]){ uint16_t d=(uint16_t)(tnow()-tKS_ticks[ii]); if(d>ARM_TIMEOUT_TICKS){ armedKS[ii]=false; tKS_ticks[ii]=0; }}
    if(armedKF[ii]){ uint16_t d=(uint16_t)(tnow()-tKF_ticks[ii]); if(d>ARM_TIMEOUT_TICKS){ armedKF[ii]=false; tKF_ticks[ii]=0; }}
  }
#endif

  if(any){ MidiUSB.flush(); digitalWrite(LED,HIGH); delay(1); digitalWrite(LED,LOW); }
}