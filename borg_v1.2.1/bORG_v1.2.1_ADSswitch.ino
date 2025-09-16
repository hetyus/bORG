// SPDX-License-Identifier: GPL-3.0-or-later
/*******************************************************
 * bORG v1.2.1 (I2C + ADS toggle build) – Korg Modwave MKI → Arduino Pro Micro USB-MIDI
 * Target: Pro Micro 3.3 V (ATmega32U4)
 *
 * === Build-time switches ===
 *   #define USE_I2C 0   → NO I2C (no MCP/ADS). FN on D15. Scanner + SysEx only.
 *   #define USE_I2C 1   → WITH I2C (MCP23017 @0x20, ADS1115 @0x48), as designed.
 *   #define USE_ADS  0   → Do not compile/link ADS code (no joystick, no PB/CC1). No MIDI spam.
 *   #define USE_ADS  1   → Compile ADS joystick (X→PitchBend, Y→ModWheel CC1).
 *
 * I²C wiring (when USE_I2C=1):
 *   SDA=D2, SCL=D3. MCP23017 @0x20: GPA0..4 inputs (sustain/extra/oct+/oct-/FN), GPB0..5 RGB LEDs.
 *   ADS1115 @0x48: A0=Joy X (Pitch Bend), A1=Joy Y (ModWheel).
 *
 * SysEx (0x7D non-commercial):
 *  v1.0: F0 7D 01 cc F7 (curve), 02 (cal start), 03 (cal stop), 04 (save), 05 (reset), 06 vv (fixedVel), 0A (status)
 *  v1.1: F0 7D 07 cc F7 (MIDI ch 1..16), 08 pp (Program Change 0..127)
 *
 * License: GPL-3.0-or-later. KORG and KORG Modwave are trademarks of KORG Inc. No warranty.
 *******************************************************/

// ===== Build flags =====
#define USE_I2C 1            // 0: no MCP/ADS; 1: MCP23017 (and optional ADS1115) via I2C
#define USE_ADS 0            // 0: ADS code fully disabled; 1: enable ADS joystick
#define IGNORE_FN_FOR_TEST 0 // 1: ignore FN button entirely
#define SHORT_FOCUS_WAIT_US 6000

// ===== Tunables (timing / sampling) =====
#ifndef MIN_DT_TICKS
  #define MIN_DT_TICKS 400     // ≈200 µs (Timer1 @ /8 → ~0.5 µs/tick)
#endif
#ifndef COL_SETTLE_US
  #define COL_SETTLE_US 150    // column driver settle (µs)
#endif
#ifndef SAMPLES
  #define SAMPLES 3            // oversample count per read
#endif
#ifndef MAJ
  #define MAJ 2                // majority threshold
#endif
#ifndef SAMPLE_DELAY_US
  #define SAMPLE_DELAY_US 0    // inter-sample delay (µs); keep 0 unless needed
#endif

#include <Arduino.h>
#include <MIDIUSB.h>
#include <EEPROM.h>
#include <math.h>

#if USE_I2C
  #include <Wire.h>
  #include <Adafruit_MCP23X17.h>
  Adafruit_MCP23X17 mcp;
#endif

#if USE_I2C && USE_ADS
  #include <Adafruit_ADS1X15.h>
  Adafruit_ADS1115 ads;
  static bool g_ads_ok = false;
#else
  static const bool g_ads_ok = false; // compile-time false when ADS disabled
#endif

// ===== Pins (adjust to your 1.2.1 sketch if different) =====
const uint8_t KS_PINS[4] = {14,5,7,9};     // KS0..KS3 (LOW=active)
const uint8_t KF_PINS[4] = {A0,A1,A2,A3};  // KF0..KF3 (LOW=active)
const uint8_t KBD0=4, KBD1=6, KBD2=8, KBD3=10, KBD4=16;
const uint8_t LED=17;                       // onboard/extern status LED
const uint8_t PIN_FN=15;                    // FN when USE_I2C=0

// MCP pins (when I2C is used)
#if USE_I2C
  #define MCP_ADDR 0x20
  #define MCP_FN     4
  #define MCP_SUSTAIN 0
  #define MCP_EXTRA   1
  #define MCP_OCT_UP  2
  #define MCP_OCT_DN  3
  #define MCP_UP_R    8
  #define MCP_UP_G    9
  #define MCP_UP_B    10
  #define MCP_DOWN_R  11
  #define MCP_DOWN_G  12
  #define MCP_DOWN_B  13
#endif

// ===== Scan/debounce / velocity =====
const bool     USE_NOTE_OFF_VELOCITY = true;
const uint8_t  RELEASE_DEBOUNCE_MS   = 3;
const uint8_t  MIN_NOTE_LEN_MS       = 8;

// ===== MIDI / mapping =====
uint8_t MIDI_CH = 1;
const uint8_t MIN_VEL=1, MAX_VEL=127;
uint8_t NOTE_BASE=48; // C3=48
int8_t  octaveShift=0;

// ===== Curves & calibration =====
enum Curve:uint8_t{CURVE_LINEAR, CURVE_FLAT, CURVE_STEEP, CURVE_PIANO, CURVE_SYNTH, CURVE_ORGAN_FIXED, CURVE_SOFT, CURVE_HARD};
float   dt_fast_us=600.0f, dt_slow_us=20000.0f; // learned bounds
Curve   currentCurve=CURVE_LINEAR;
uint8_t fixedVelocity=100;
bool    calib_locked=true;
uint16_t learn_count=0;

// ===== Note map (PLACEHOLDER) =====
// Paste your exact NOTE_KS and NOTE_KF arrays from your 1.2.1 sketch.
struct Contact{uint8_t panel,col,row;};
// Example declarations (content must be supplied from your existing sketch):
const Contact NOTE_KS[37] = {/* TODO: paste actual KS mapping */};
const Contact NOTE_KF[37] = {/* TODO: paste actual KF mapping */};
int8_t REV_KS[2][8][4], REV_KF[2][8][4];

// ===== State =====
bool lastKS[2][8][4]={{ {0} }}, lastKF[2][8][4]={{ {0} }};
uint16_t tKS_ticks[37]={0}, tKF_ticks[37]={0};
bool noteOnSent[37]={0};
uint32_t noteOnMs[37]={0};
bool releaseArm[37]={0};
uint16_t tFirstReleaseTicks[37]={0};
uint32_t bothOpenSinceMs[37]={0};

// ===== Timer1 helpers =====
static inline void timer1_init(){ TCCR1A=0; TCCR1B=_BV(CS11); TCNT1=0; }
static inline uint16_t tnow(){ return TCNT1; }

// ===== EEPROM =====
struct Persist{uint32_t magic; float dt_fast,dt_slow; uint8_t curve,locked,fixedVel,_pad[5];};
const uint32_t MAGIC=0x4B4D5744UL; const int EEPROM_ADDR=0;
void saveEEP(){ Persist p{MAGIC,dt_fast_us,dt_slow_us,(uint8_t)currentCurve,(uint8_t)(calib_locked?1:0),fixedVelocity,{0}}; EEPROM.put(EEPROM_ADDR,p); }
bool loadEEP(){ Persist p; EEPROM.get(EEPROM_ADDR,p); if(p.magic!=MAGIC) return false; dt_fast_us=p.dt_fast; dt_slow_us=p.dt_slow; currentCurve=(Curve)p.curve; calib_locked=(p.locked!=0); fixedVelocity=p.fixedVel? p.fixedVel:100; return true; }
void resetEEP(){ Persist p{}; EEPROM.put(EEPROM_ADDR,p); }

// ===== HW helpers =====
static inline void setEnableForPanel(uint8_t panel){ digitalWrite(KBD3, panel ? HIGH : LOW); digitalWrite(KBD4, LOW); }
static inline void selCol(uint8_t y){ digitalWrite(KBD0,(y&1)?HIGH:LOW); digitalWrite(KBD1,(y&2)?HIGH:LOW); digitalWrite(KBD2,(y&4)?HIGH:LOW); delayMicroseconds(COL_SETTLE_US); }
static inline bool rd(uint8_t pin){ uint8_t hit=0; for(uint8_t i=0;i<SAMPLES;i++){ if(digitalRead(pin)==LOW) hit++; if(SAMPLE_DELAY_US) delayMicroseconds(SAMPLE_DELAY_US);} return hit>=MAJ; }

// ===== MIDI helpers =====
static inline void midiOn(uint8_t note,uint8_t vel){ midiEventPacket_t p{0x09,(uint8_t)(0x90|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p); }
static inline void midiOffV(uint8_t note,uint8_t vel){ midiEventPacket_t p{0x08,(uint8_t)(0x80|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p); }
static inline void sendCC(uint8_t cc,uint8_t val){ midiEventPacket_t p{0x0B,(uint8_t)(0xB0|(MIDI_CH-1)),cc,val}; MidiUSB.sendMIDI(p); }
static inline void sendPB(int16_t bend){ uint16_t u=(uint16_t)(bend+8192); uint8_t lsb=u&0x7F, msb=(u>>7)&0x7F; midiEventPacket_t p{0x0E,(uint8_t)(0xE0|(MIDI_CH-1)),lsb,msb}; MidiUSB.sendMIDI(p); }

// ===== Curves =====
static inline float curve_piano(float x){ const float k=6.0f; return 1.0f/(1.0f+expf(-k*(0.5f-x))); }
static inline float applyCurve01(float x, Curve c){
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
  float y=applyCurve01(x,currentCurve); if(y<0)y=0; if(y>1)y=1;
  uint8_t v=(uint8_t)lroundf(y*(MAX_VEL-MIN_VEL)+MIN_VEL); if(v<MIN_VEL)v=MIN_VEL; if(v>MAX_VEL)v=MAX_VEL; return v;
}

// ===== Focus helper =====
static inline bool shortFocusWait(uint8_t panel,uint8_t col,uint8_t row,bool wantKS,uint16_t &tOutTicks){
  setEnableForPanel(panel); selCol(col); uint32_t us=0;
  while(us<SHORT_FOCUS_WAIT_US){ bool v=rd(wantKS?KS_PINS[row]:KF_PINS[row]); if(v){ tOutTicks=tnow(); return true; } delayMicroseconds(3); us+=3; }
  return false;
}

// ===== Rev map =====
void buildRev();

// ===== Status =====
bool g_eep_loaded=false;
const char* curveName(Curve c){ switch(c){ case CURVE_LINEAR:return "LINEAR"; case CURVE_FLAT:return "FLAT"; case CURVE_STEEP:return "STEEP"; case CURVE_PIANO:return "PIANO"; case CURVE_SYNTH:return "SYNTH"; case CURVE_ORGAN_FIXED:return "ORGAN_FIXED"; case CURVE_SOFT:return "SOFT"; case CURVE_HARD:return "HARD"; default:return "UNKNOWN"; } }
void printStatus(){
  Serial.println(F("=== bORG v1.2.1 (I2C/ADS toggle) STATUS ==="));
  Serial.print(F("EEPROM loaded: ")); Serial.println(g_eep_loaded?F("YES"):F("NO"));
  Serial.print(F("Calib locked: ")); Serial.println(calib_locked?F("YES"):F("NO"));
  Serial.print(F("Curve: ")); Serial.println(curveName(currentCurve));
  Serial.print(F("Fixed velocity: ")); Serial.println(fixedVelocity);
  Serial.print(F("dt_fast_us: ")); Serial.println(dt_fast_us);
  Serial.print(F("dt_slow_us: ")); Serial.println(dt_slow_us);
  Serial.print(F("MIN_DT_TICKS: ")); Serial.println((int)MIN_DT_TICKS);
  Serial.print(F("COL_SETTLE_US: ")); Serial.println((int)COL_SETTLE_US);
  Serial.print(F("SAMPLES/MAJ: ")); Serial.print((int)SAMPLES); Serial.print('/'); Serial.println((int)MAJ);
  Serial.print(F("USE_I2C: ")); Serial.println((int)USE_I2C);
  Serial.print(F("USE_ADS: ")); Serial.println((int)USE_ADS);
}

// ===== I/O utils =====
#if USE_I2C
  inline bool mcpRead(uint8_t pin){ return (mcp.digitalRead(pin)==LOW); }
  inline bool fnActive(){ return IGNORE_FN_FOR_TEST ? false : mcpRead(MCP_FN); }
#else
  inline bool fnActive(){ return IGNORE_FN_FOR_TEST ? false : (digitalRead(PIN_FN)==LOW); }
#endif

// ===== Joystick (ADS) =====
static int16_t lastPB=0; static uint8_t lastCC1=255; // latch to avoid spam
void processJoystick(){
#if USE_I2C && USE_ADS
  static uint32_t last=0; if(millis()-last<5) return; last=millis();
  if(!g_ads_ok) return; // no messages when ADS is not present
  int16_t vx=ads.readADC_SingleEnded(0), vy=ads.readADC_SingleEnded(1);
  const int16_t MID=16384, DZ=800, SPAN=16384;
  int32_t d=(int32_t)vx-MID; if(abs(d)<=DZ) d=0; else if(d>0) d-=DZ; else d+=DZ;
  long pbL=(long)d*8191/(SPAN-DZ); if(pbL>8191) pbL=8191; if(pbL<-8191) pbL=-8191;
  uint8_t cc1=(uint8_t)constrain((long)vy*127/32767,0,127);
  if((int16_t)pbL!=lastPB){ sendPB((int16_t)pbL); lastPB=(int16_t)pbL; }
  if(cc1!=lastCC1){ sendCC(1,cc1); lastCC1=cc1; }
#else
  // ADS disabled at build time: do nothing (no PB/CC1 output, no spam).
  return;
#endif
}

// ===== Buttons (MCP) =====
void handleButtons(){
#if USE_I2C
  static uint32_t last=0; if(millis()-last<10) return; last=millis();
  // sustain
  static bool sustPrev=false; bool sustNow=mcpRead(MCP_SUSTAIN); if(sustNow!=sustPrev){ sendCC(64, sustNow?127:0); sustPrev=sustNow; }
  // extra
  static bool extraPrev=false; bool extraNow=mcpRead(MCP_EXTRA); if(extraNow!=extraPrev){ sendCC(67, extraNow?127:0); extraPrev=extraNow; }
  // octave
  static bool upPrev=false, dnPrev=false; bool upNow=mcpRead(MCP_OCT_UP), dnNow=mcpRead(MCP_OCT_DN);
  if(upNow && !upPrev){ if(octaveShift<3){ octaveShift++; } }
  if(dnNow && !dnPrev){ if(octaveShift>-3){ octaveShift--; } }
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

// ===== FN shortcuts / SysEx =====
void handleFnShortcutByNote(uint8_t midiNote){
  if(midiNote>=NOTE_BASE && midiNote<=NOTE_BASE+15){ MIDI_CH=1+(midiNote-NOTE_BASE); return; }
  if(midiNote>=NOTE_BASE+16 && midiNote<=NOTE_BASE+23){ uint8_t cc=midiNote-(NOTE_BASE+16); if(cc<=7) currentCurve=(Curve)cc; return; }
  switch(midiNote){
    case 60: calib_locked=false; learn_count=0; break; // C4 = Cal START
    case 62: calib_locked=true; break;                 // D4 = Cal STOP/LOCK
    case 64: saveEEP(); break;                         // E4 = Save
    case 65: resetEEP(); dt_fast_us=600.0f; dt_slow_us=20000.0f; currentCurve=CURVE_LINEAR; calib_locked=true; fixedVelocity=100; break; // F4 reset
    case 67: printStatus(); break;                     // G4 = Status
    default: break;
  }
}

void handleSysExBuffer(uint8_t *buf,uint8_t len){
  if(len<4||buf[0]!=0xF0||buf[len-1]!=0xF7||buf[1]!=0x7D) return;
  // Parse mini command set
  if(len>=4){ uint8_t cmd=buf[2];
    switch(cmd){
      case 0x01: if(len>=5){ uint8_t c=buf[3]; if(c<=7) currentCurve=(Curve)c; } break;
      case 0x02: calib_locked=false; learn_count=0; break;
      case 0x03: calib_locked=true; break;
      case 0x04: saveEEP(); break;
      case 0x05: resetEEP(); dt_fast_us=600.0f; dt_slow_us=20000.0f; currentCurve=CURVE_LINEAR; calib_locked=true; fixedVelocity=100; break;
      case 0x06: if(len>=5){ fixedVelocity=buf[3]; } break;
      case 0x07: if(len>=5){ uint8_t ch=buf[3]; if(ch>=1&&ch<=16) MIDI_CH=ch; } break;
      case 0x08: if(len>=5){ uint8_t pc=buf[3]; midiEventPacket_t p{0x0C,(uint8_t)(0xC0|(MIDI_CH-1)),pc,0}; MidiUSB.sendMIDI(p);} break;
      case 0x0A: printStatus(); break;
      default: break;
    }
  }
}

static uint8_t sysexBuf[16]; static uint8_t sysexLen=0;
void pollMIDIin(){
  midiEventPacket_t rx = MidiUSB.read(); if(rx.header==0) return;
  if((rx.byte1 & 0xF0)==0x90 && (rx.byte3>0)){ /* NoteOn in */ handleFnShortcutByNote(rx.byte2); }
  if(rx.byte1==0xF0){ // crude SysEx collector up to 16B
    if(sysexLen<sizeof(sysexBuf)) sysexBuf[sysexLen++]=rx.byte1;
    if(rx.byte1==0xF7){ handleSysExBuffer(sysexBuf, sysexLen); sysexLen=0; }
  }
}

// ===== Setup & Loop =====
void setup(){
  pinMode(LED,OUTPUT); digitalWrite(LED,LOW);
  pinMode(PIN_FN, INPUT_PULLUP);
  for(uint8_t i=0;i<4;i++){ pinMode(KS_PINS[i],INPUT_PULLUP); pinMode(KF_PINS[i],INPUT_PULLUP); }
  pinMode(KBD0,OUTPUT); pinMode(KBD1,OUTPUT); pinMode(KBD2,OUTPUT); pinMode(KBD3,OUTPUT); pinMode(KBD4,OUTPUT);
  Serial.begin(115200); while(!Serial){;}
  timer1_init(); buildRev();
  if(loadEEP()) g_eep_loaded=true; else { /* keep defaults */ }

#if USE_I2C
  Wire.begin(); Wire.setClock(100000);
  if(mcp.begin_I2C(MCP_ADDR)){
    for(uint8_t i=0;i<=4;i++) mcp.pinMode(i, INPUT_PULLUP); // GPA0..4 inputs
    for(uint8_t i=8;i<=13;i++){ mcp.pinMode(i, OUTPUT); mcp.digitalWrite(i,LOW); } // RGB LED outputs
  }
  #if USE_ADS
    if(ads.begin(0x48)){ ads.setGain(GAIN_ONE); g_ads_ok=true; }
  #endif
#endif
}

static inline bool tryComputeVelAndTrigger(uint8_t idx,uint16_t tA,uint16_t tB){
  if(tA==0||tB==0) return false;
  uint16_t dt=(uint16_t)(tA-tB); if((int16_t)dt<0) dt=(uint16_t)(tB-tA);
  if(dt<MIN_DT_TICKS) return false;
  uint8_t vel=vel_from_dt_ticks(dt);
  if(fnActive()){ handleFnShortcutByNote((uint8_t)(NOTE_BASE+idx+12*octaveShift)); }
  else{
    midiOn((uint8_t)(NOTE_BASE+idx+12*octaveShift), vel);
    noteOnSent[idx]=true; noteOnMs[idx]=millis(); releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; updateCalib((uint32_t)dt>>1);
  }
  return true;
}

void heartbeat(){ static uint32_t last=0; if(millis()-last<250) return; last=millis(); digitalWrite(LED,!digitalRead(LED)); }

void loop(){
  heartbeat();
  pollMIDIin();
  handleButtons();
  processJoystick();

  bool any=false;
  for(uint8_t panel=0; panel<2; panel++){
    setEnableForPanel(panel);
    for(uint8_t col=0; col<8; col++){
      selCol(col);
      for(uint8_t row=0; row<4; row++){
        int8_t iKS=REV_KS[panel][col][row]; int8_t iKF=REV_KF[panel][col][row];
        if(iKS<0 || iKF<0) continue;
        bool vKS=rd(KS_PINS[row]); bool vKF=rd(KF_PINS[row]);
        uint8_t idx=(uint8_t)iKS; // same index mapping for KS/KF
        // Press
        if(vKS && vKF){
          if(!noteOnSent[idx]){
            uint16_t tA=0,tB=0; bool okA=shortFocusWait(panel,col,row,true,tA); bool okB=shortFocusWait(panel,col,row,false,tB);
            if(okA && okB){ if(tryComputeVelAndTrigger(idx,tA,tB)) any=true; }
          }
        }
        // Release
        else if(!vKS && !vKF){
          if(noteOnSent[idx]){
            if(!releaseArm[idx]){ releaseArm[idx]=true; tFirstReleaseTicks[idx]=tnow(); bothOpenSinceMs[idx]=millis(); }
            auto processOffFor=[&](int8_t which,bool vks,bool vkf){
              if(which<0) return; if((millis()-bothOpenSinceMs[idx])<RELEASE_DEBOUNCE_MS) return;
              uint8_t offVel=MIN_VEL; // default min off-vel
              if(USE_NOTE_OFF_VELOCITY){ int32_t d=(int16_t)tnow()-(int16_t)tFirstReleaseTicks[idx]; if(d<0) d=-d; offVel=vel_from_dt_ticks((uint16_t)d); }
              midiOffV((uint8_t)(NOTE_BASE+idx+12*octaveShift), offVel);
              noteOnSent[idx]=false; tKS_ticks[idx]=0; tKF_ticks[idx]=0; releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; bothOpenSinceMs[idx]=0; any=true;
            };
            processOffFor(iKS,vKS,vKF);
            processOffFor(iKF,vKS,vKF);
          } else {
            bothOpenSinceMs[idx]=0;
          }
        }
      }
    }
  }
  if(any){ MidiUSB.flush(); digitalWrite(LED,HIGH); delay(1); digitalWrite(LED,LOW); }
}

// ====== NOTE MAP DEFINITIONS ======
// Paste your exact NOTE_KS and NOTE_KF arrays content below and un-comment the reverse-map builder.

void buildRev(){
  for(uint8_t p=0;p<2;p++) for(uint8_t y=0;y<8;y++) for(uint8_t r=0;r<4;r++){ REV_KS[p][y][r]=-1; REV_KF[p][y][r]=-1; }
  for(uint8_t i=0;i<37;i++){
    // REV_KS[NOTE_KS[i].panel][NOTE_KS[i].col][NOTE_KS[i].row]=i;
    // REV_KF[NOTE_KF[i].panel][NOTE_KF[i].col][NOTE_KF[i].row]=i;
  }
}
