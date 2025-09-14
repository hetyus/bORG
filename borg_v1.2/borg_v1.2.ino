// SPDX-License-Identifier: GPL-3.0-or-later
/*******************************************************
 * bORG v1.2 – Korg Modwave MKI -> Arduino Pro Micro USB-MIDI
 *
 * Hardware (3.3 V only):
 *  - Keybed scan unchanged (KS/KF + 74HC138 on KBD0..4).
 *  - I²C bus: D2=SDA, D3=SCL shared.
 *  - MCP23017 (Adafruit MCP23X17 lib) @0x20:
 *      GPA0  Sustain (CC64) input, INPUT_PULLUP, active LOW
 *      GPA1  Extra   (CC67) input, INPUT_PULLUP, active LOW
 *      GPA2  Octave UP     input, INPUT_PULLUP, active LOW
 *      GPA3  Octave DOWN   input, INPUT_PULLUP, active LOW
 *      GPA4  FN button     input, INPUT_PULLUP, active LOW
 *      GPB0..2  RGB LED – DOWN (R,G,B) anodes via 220–330 Ω
 *      GPB3..5  RGB LED – UP   (R,G,B) anodes via 220–330 Ω
 *      LED cathodes -> GND (common cathode)
 *  - ADS1115 @0x48:
 *      A0 <- Joystick VRx → Pitch Bend
 *      A1 <- Joystick VRy → Mod Wheel (CC1)
 *
 * SysEx (0x7D non-commercial):
 *  v1.0:
 *    F0 7D 01 cc F7        Select velocity curve (cc=0..7)
 *    F0 7D 02 F7           Calibration start (unlock)
 *    F0 7D 03 F7           Calibration lock (stop)
 *    F0 7D 04 F7           Save calibration + settings to EEPROM
 *    F0 7D 05 F7           Factory reset (defaults)
 *    F0 7D 06 vv F7        Set fixed velocity (1..127)
 *    F0 7D 0A F7           Print STATUS to Serial Monitor
 *  v1.1:
 *    F0 7D 07 cc F7        Set MIDI channel (cc=1..16, hex byte)
 *    F0 7D 08 pp F7        Program Change (pp=0..127)
 *  v1.2.1:
 *    FN + keys (hostless shortcuts):
 *      – MIDI channel: 16 keys from NOTE_BASE → ch 1..16
 *      – Velocity curve: next 8 keys → 0..7
 *      – C4 → Calib START, D4 → Calib STOP, E4 → SAVE, F4 → RESET, G4 → STATUS
 *    Octave shift buttons (−3..+3), LED color feedback:
 *      ±1=GREEN, ±2=YELLOW (R+G), ±3=RED; 0 → LEDs off
 *
 * License: GPL-3.0-or-later. KORG és KORG Modwave védjegyek a KORG Inc. tulajdonai.
 * Ez egy független, közösségi projekt, garancia nélkül.
 *******************************************************/

#include <Arduino.h>
#include <MIDIUSB.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>   // új könyvtár
#include <Adafruit_ADS1X15.h>

/*** PINS – SERVICE MANUAL SZERINTI NEVEK (változatlan 1.0-ból) ***/
// KS0 átrakva D3-ról D14-re (I2C felszabadítva: D2=SDA, D3=SCL)
const uint8_t KS_PINS[4] = {14, 5, 7, 9};     // KS0..KS3  (INPUT, külső 10k -> 3.3V)
const uint8_t KF_PINS[4] = {A0, A1, A2, A3};  // KF0..KF3  (INPUT, külső 10k -> 3.3V)

// 74HC138 cím- és enable-vonalak
const uint8_t KBD0 = 4;   // A (LSB)
const uint8_t KBD1 = 6;   // B
const uint8_t KBD2 = 8;   // C (MSB)
const uint8_t KBD3 = 10;  // /G2A (panel-váltó) – 0:2665, 1:2663
const uint8_t KBD4 = 16;  // /G2B (közös enable) – mindig LOW

const uint8_t LED  = 17;  // TX LED (csak villantásra)

/*** I2C eszközök ***/
Adafruit_MCP23X17 mcp;     // @0x20 (MCP23017 az új libben)
Adafruit_ADS1115  ads;     // @0x48
bool g_ads_ok = false;

/*** MCP23017 kiosztás ***/
// Inputs (GPA0..4):
const uint8_t MCP_SUSTAIN = 0; // GPA0
const uint8_t MCP_EXTRA   = 1; // GPA1
const uint8_t MCP_OCT_UP  = 2; // GPA2
const uint8_t MCP_OCT_DN  = 3; // GPA3
const uint8_t MCP_FN      = 4; // GPA4
// Outputs (GPB0..5):
const uint8_t MCP_DOWN_R = 8+0; // GPB0
const uint8_t MCP_DOWN_G = 8+1; // GPB1
const uint8_t MCP_DOWN_B = 8+2; // GPB2
const uint8_t MCP_UP_R   = 8+3; // GPB3
const uint8_t MCP_UP_G   = 8+4; // GPB4
const uint8_t MCP_UP_B   = 8+5; // GPB5

/*** SCAN / DEBOUNCE / FOCUS ***/
// erősebb bemeneti szűrés (akkord/rángás ellen)
const uint8_t  SAMPLES=3, MAJ=2;     // ha zajos: 3/2
const uint16_t SAMPLE_DELAY_US=0;
const uint16_t COL_SETTLE_US=120;
const uint16_t FOCUS_TIMEOUT_TICKS = 60000; // 0.5us/tick → ~30 ms
const uint8_t  FOCUS_POLL_US       = 3;

/*** RELEASE / OFF kezelés ***/
const uint8_t  RELEASE_DEBOUNCE_MS = 3;
const uint8_t  MIN_NOTE_LEN_MS     = 8;
const bool     USE_NOTE_OFF_VELOCITY = true;

/*** MIDI ***/
uint8_t  MIDI_CH=1;
const uint8_t  MIN_VEL=1, MAX_VEL=127;
uint8_t  NOTE_BASE=48;              // C3
int8_t   octaveShift = 0;           // −3..+3

/*** Görbék / Presetek ***/
enum Curve : uint8_t {
  CURVE_LINEAR=0, CURVE_FLAT=1, CURVE_STEEP=2, CURVE_PIANO=3,
  CURVE_SYNTH=4, CURVE_ORGAN_FIXED=5, CURVE_SOFT=6, CURVE_HARD=7
};
float   dt_fast_us = 600.0f;
float   dt_slow_us = 20000.0f;
Curve   currentCurve = CURVE_LINEAR;
uint8_t fixedVelocity = 100;
bool    calib_locked = true;
uint16_t learn_count = 0;

/*** NOTE MAP (mint 1.0) ***/
struct Contact { uint8_t panel,col,row; };
const Contact NOTE_KS[37] = {
  {1,1,3},{1,1,2},{1,1,1},{1,1,0},
  {1,0,3},{1,0,2},{1,0,1},{1,0,0},
  {0,7,3},{0,7,2},{0,7,1},{0,7,0},
  {0,6,0},
  {0,5,3},{0,5,2},{0,5,1},{0,5,0},
  {0,4,3},{0,4,2},{0,4,1},{0,4,0},
  {0,3,3},{0,3,2},{0,3,1},{0,3,0},
  {0,2,3},{0,2,2},{0,2,1},{0,2,0},
  {0,1,3},{0,1,2},{0,1,1},{0,1,0},
  {0,0,3},{0,0,2},{0,0,1},{0,0,0}
};
const Contact NOTE_KF[37] = {
  {1,1,3},{1,1,2},{1,1,1},{1,1,0},
  {1,0,3},{1,0,2},{1,0,1},{1,0,0},
  {0,7,3},{0,7,2},{0,7,1},{0,7,0},
  {0,6,0},
  {0,5,3},{0,5,2},{0,5,1},{0,5,0},
  {0,4,3},{0,4,2},{0,4,1},{0,4,0},
  {0,3,3},{0,3,2},{0,3,1},{0,3,0},
  {0,2,3},{0,2,2},{0,2,1},{0,2,0},
  {0,1,3},{0,1,2},{0,1,1},{0,1,0},
  {0,0,3},{0,0,2},{0,0,1},{0,0,0}
};

/*** REV MAP ***/
int8_t REV_KS[2][8][4];
int8_t REV_KF[2][8][4];

/*** Állapot ***/
bool     ksDown[37]={0};
bool     lastKS[2][8][4] = {{{0}}};
bool     lastKF[2][8][4] = {{{0}}};
uint16_t tKS_ticks[37]={0}, tKF_ticks[37]={0};
bool     noteOnSent[37]={0};

// NoteOff-hoz
uint32_t noteOnMs[37]={0};
bool     releaseArm[37]={0};
uint16_t tFirstReleaseTicks[37]={0};

/*** Timer1: /8 prescaler → 2 MHz (0.5 us/tick) ***/
static inline void timer1_init(){ TCCR1A=0; TCCR1B=_BV(CS11); TCNT1=0; }
static inline uint16_t tnow(){ return TCNT1; }

/*** EEPROM ***/
struct Persist {
  uint32_t magic;    // 'KMWD'
  float    dt_fast;
  float    dt_slow;
  uint8_t  curve;
  uint8_t  locked;
  uint8_t  fixedVel;
  uint8_t  _pad[5];
};
const uint32_t MAGIC = 0x4B4D5744UL;
const int EEPROM_ADDR = 0;

void saveEEP(){
  Persist p;
  p.magic    = MAGIC;
  p.dt_fast  = dt_fast_us;
  p.dt_slow  = dt_slow_us;
  p.curve    = (uint8_t)currentCurve;
  p.locked   = calib_locked ? 1 : 0;
  p.fixedVel = fixedVelocity;
  EEPROM.put(EEPROM_ADDR, p);
}
bool loadEEP(){
  Persist p; EEPROM.get(EEPROM_ADDR, p);
  if(p.magic != MAGIC) return false;
  dt_fast_us    = p.dt_fast;
  dt_slow_us    = p.dt_slow;
  currentCurve  = (Curve)p.curve;
  calib_locked  = p.locked!=0;
  fixedVelocity = p.fixedVel ? p.fixedVel : 100;
  return true;
}
void resetEEP(){ Persist p{}; EEPROM.put(EEPROM_ADDR, p); }

/*** HW segédek ***/
static inline void setEnableForPanel(uint8_t panel){
  digitalWrite(KBD3, panel ? HIGH : LOW); // 0=2665, 1=2663
  digitalWrite(KBD4, LOW);
}
static inline void selCol(uint8_t y){
  digitalWrite(KBD0,(y&1)?HIGH:LOW);
  digitalWrite(KBD1,(y&2)?HIGH:LOW);
  digitalWrite(KBD2,(y&4)?HIGH:LOW);
  delayMicroseconds(COL_SETTLE_US);
}
static inline bool rd(uint8_t pin){
  uint8_t hit=0;
  for(uint8_t i=0;i<SAMPLES;i++){
    if(digitalRead(pin)==LOW) hit++;
    if(SAMPLE_DELAY_US) delayMicroseconds(SAMPLE_DELAY_US);
  }
  return hit>=MAJ;
}

/*** MIDI segédek ***/
static inline void midiOn(uint8_t note,uint8_t vel){ midiEventPacket_t p={0x09,(uint8_t)(0x90|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p); }
static inline void midiOff(uint8_t note){ midiEventPacket_t p={0x08,(uint8_t)(0x80|(MIDI_CH-1)),note,0}; MidiUSB.sendMIDI(p); }
static inline void midiOffV(uint8_t note,uint8_t vel){ midiEventPacket_t p={0x08,(uint8_t)(0x80|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p); }
static inline void sendCC(uint8_t cc, uint8_t val){ midiEventPacket_t p={0x0B,(uint8_t)(0xB0|(MIDI_CH-1)),cc,val}; MidiUSB.sendMIDI(p); }
static inline void sendPB(int16_t bend){ uint16_t u = (uint16_t)(bend + 8192); uint8_t lsb=u&0x7F, msb=(u>>7)&0x7F; midiEventPacket_t p={0x0E,(uint8_t)(0xE0|(MIDI_CH-1)),lsb,msb}; MidiUSB.sendMIDI(p); }

/*** Görbék ***/
static inline float curve_piano(float x){
  const float k = 6.0f;
  return 1.0f / (1.0f + expf(-k*(0.5f - x)));
}
static inline float applyCurve01(float x, Curve c){
  if(x<0) x=0; if(x>1) x=1;
  switch(c){
    case CURVE_LINEAR:       return 1.0f - x;
    case CURVE_FLAT:         return 1.0f - powf(x, 1.6f);
    case CURVE_STEEP:        return 1.0f - powf(x, 0.7f);
    case CURVE_PIANO:        return curve_piano(x);
    case CURVE_SYNTH:        return 1.0f - (1.0f - expf(-3.0f*x))/(1.0f - expf(-3.0f));
    case CURVE_ORGAN_FIXED:  return -1.0f; // külön kezeljük
    case CURVE_SOFT:         return 1.0f - powf(x, 2.2f);
    case CURVE_HARD:         return 1.0f - powf(x, 0.5f);
    default:                 return 1.0f - x;
  }
}

/*** Kalibráció ***/
static inline void updateCalib(uint32_t dt_us){
  if (calib_locked) return;
  const float ALPHA_FAST = 0.25f;
  const float ALPHA_SLOW = 0.18f;
  const float DRIFT      = 0.02f;

  float eF = (float)dt_us - dt_fast_us;
  dt_fast_us += (eF < 0 ? ALPHA_FAST*eF : DRIFT*eF);
  if(dt_fast_us < 150.0f) dt_fast_us = 150.0f;

  float eS = (float)dt_us - dt_slow_us;
  dt_slow_us += (eS > 0 ? ALPHA_SLOW*eS : DRIFT*eS);
  if(dt_slow_us > 40000.0f) dt_slow_us = 40000.0f;

  if(dt_slow_us - dt_fast_us < 3000.0f) dt_slow_us = dt_fast_us + 3000.0f;
}

/*** dt_ticks → velocity ***/
static inline uint8_t vel_from_dt_ticks(uint16_t dt_ticks){
  if(currentCurve == CURVE_ORGAN_FIXED){
    uint8_t v = fixedVelocity; if(v<MIN_VEL) v=MIN_VEL; if(v>MAX_VEL) v=MAX_VEL; return v;
  }
  uint32_t dt_us = (uint32_t)dt_ticks >> 1; // 0.5us/tick
  float fast=dt_fast_us, slow=dt_slow_us;
  float x = (float)((int32_t)dt_us - (int32_t)fast) / (float)(slow-fast);
  if(x<0) x=0; if(x>1) x=1;
  float y = applyCurve01(x, currentCurve);
  if(y<0) y=0; if(y>1) y=1;
  uint8_t v=(uint8_t)lroundf(y*(MAX_VEL-MIN_VEL)+MIN_VEL);
  if(v<MIN_VEL) v=MIN_VEL; if(v>MAX_VEL) v=MAX_VEL;
  return v;
}

/*** Velocity-robosztussági helper (NINCS blokkoló várakozás) ***/
static inline bool tryComputeVelAndTrigger(uint8_t physIndex, uint16_t tA, uint16_t tB){
  if(tA==0 || tB==0) return false;             // még nincs meg mindkettő
  uint16_t dt = (uint16_t)(tA - tB);
  if((int16_t)dt < 0) dt = (uint16_t)(tB - tA);

  // Védő küszöb – túl kicsi dt = valószínű időzítési/bounce hiba
  const uint16_t MIN_DT_TICKS = 400; // ~200 µs (Timer1 0.5 µs/tick)
  if(dt < MIN_DT_TICKS) return false;

  uint8_t vel = vel_from_dt_ticks(dt);

  if(fnActive()){
    handleFnShortcutByNote((uint8_t)(NOTE_BASE + physIndex + 12*octaveShift));
  }else{
    midiOn((uint8_t)(NOTE_BASE + physIndex + 12*octaveShift), vel);
    noteOnSent[physIndex]=true;
    noteOnMs[physIndex]=millis(); releaseArm[physIndex]=false; tFirstReleaseTicks[physIndex]=0;
    updateCalib((uint32_t)dt >> 1);
  }
  return true;
}

/*** REV MAP ***/
void buildRev(){
  for(uint8_t p=0;p<2;p++) for(uint8_t y=0;y<8;y++) for(uint8_t r=0;r<4;r++){ REV_KS[p][y][r]=-1; REV_KF[p][y][r]=-1; }
  for(uint8_t i=0;i<37;i++){
    REV_KS[ NOTE_KS[i].panel ][ NOTE_KS[i].col ][ NOTE_KS[i].row ] = i;
    REV_KF[ NOTE_KF[i].panel ][ NOTE_KF[i].col ][ NOTE_KF[i].row ] = i;
  }
}

/*** STATUS helper-ek ***/
bool g_eep_loaded = false;

const char* curveName(Curve c){
  switch(c){
    case CURVE_LINEAR: return "LINEAR";
    case CURVE_FLAT: return "FLAT";
    case CURVE_STEEP: return "STEEP";
    case CURVE_PIANO: return "PIANO";
    case CURVE_SYNTH: return "SYNTH";
    case CURVE_ORGAN_FIXED: return "ORGAN_FIXED";
    case CURVE_SOFT: return "SOFT";
    case CURVE_HARD: return "HARD";
    default: return "UNKNOWN";
  }
}

void printStatus(){
  Serial.println(F("=== bORG v1.2.1 STATUS ==="));
  Serial.print(F("EEPROM loaded: ")); Serial.println(g_eep_loaded ? F("YES") : F("NO (defaults)"));
  Serial.print(F("Calib locked: ")); Serial.println(calib_locked ? F("YES") : F("NO (learning mode)"));
  Serial.print(F("dt_fast_us: ")); Serial.println(dt_fast_us, 1);
  Serial.print(F("dt_slow_us: ")); Serial.println(dt_slow_us, 1);
  Serial.print(F("Curve: ")); Serial.println(curveName(currentCurve));
  Serial.print(F("Fixed velocity (ORGAN_FIXED): ")); Serial.println(fixedVelocity);
  Serial.print(F("NOTE_BASE: ")); Serial.println(NOTE_BASE);
  Serial.print(F("MIDI_CH: ")); Serial.println(MIDI_CH);
  Serial.print(F("Octave shift: ")); Serial.println(octaveShift);
  Serial.println(F("=========================="));
}

/*** MCP23017 LED segédek ***/
void mcpSetRGB(bool up, uint8_t r, uint8_t g, uint8_t b){
  if(up){
    mcp.digitalWrite(MCP_UP_R, r?HIGH:LOW);
    mcp.digitalWrite(MCP_UP_G, g?HIGH:LOW);
    mcp.digitalWrite(MCP_UP_B, b?HIGH:LOW);
  }else{
    mcp.digitalWrite(MCP_DOWN_R, r?HIGH:LOW);
    mcp.digitalWrite(MCP_DOWN_G, g?HIGH:LOW);
    mcp.digitalWrite(MCP_DOWN_B, b?HIGH:LOW);
  }
}
void showOctaveLED(){
  mcpSetRGB(false,0,0,0); mcpSetRGB(true,0,0,0);
  if(octaveShift==0) return;
  bool up = octaveShift>0;
  uint8_t mag = abs(octaveShift); // 1..3
  uint8_t r=0,g=0,b=0;
  if(mag==1){ g=1; }
  else if(mag==2){ r=1; g=1; }   // sárga
  else { r=1; }                  // piros
  mcpSetRGB(up, r,g,b);
}

/*** ADS1115 joystick ***/
int16_t readADS(uint8_t ch){
  if(!g_ads_ok) return 0;
  return ads.readADC_SingleEnded(ch); // 0..32767
}
int16_t mapPB(int16_t v, int16_t mid, int16_t dz, int16_t span){
  int32_t d = (int32_t)v - mid;
  if(abs(d) <= dz) return 0;
  if(d>0) d -= dz; else d += dz;
  long out = (long)d * 8191 / (span - dz);
  if(out>8191) out=8191; if(out<-8191) out=-8191;
  return (int16_t)out;
}
uint8_t mapCC(int16_t v, int16_t minv, int16_t maxv){
  if(v<minv) v=minv; if(v>maxv) v=maxv;
  long out = (long)(v-minv) * 127 / (maxv-minv);
  if(out<0) out=0; if(out>127) out=127;
  return (uint8_t)out;
}
void processJoystick(){
  static uint32_t last=0;
  if(millis()-last < 5) return;
  last = millis();

  if(!g_ads_ok){ sendPB(0); sendCC(1,0); return; }
  int16_t vx = readADS(0); // VRx
  int16_t vy = readADS(1); // VRy
  const int16_t MID = 16384;
  const int16_t DZ  = 800;
  const int16_t SPAN= 16384;

  int16_t pb  = mapPB(vx, MID, DZ, SPAN);
  uint8_t cc1 = mapCC(vy, 0, 32767);

  sendPB(pb);
  sendCC(1, cc1);
}

/*** MCP input olvasás ***/
inline bool mcpRead(uint8_t pin){ return mcp.digitalRead(pin) == LOW; } // active LOW

void handleButtons(){
  static uint32_t last=0;
  if(millis()-last < 10) return;
  last = millis();

  // Sustain CC64
  static bool sustPrev=false;
  bool sustNow = mcpRead(MCP_SUSTAIN);
  if(sustNow != sustPrev){ sendCC(64, sustNow?127:0); sustPrev = sustNow; }

  // Extra CC67
  static bool extraPrev=false;
  bool extraNow = mcpRead(MCP_EXTRA);
  if(extraNow != extraPrev){ sendCC(67, extraNow?127:0); extraPrev = extraNow; }

  // Octave shift
  static bool upPrev=false, dnPrev=false;
  bool upNow = mcpRead(MCP_OCT_UP);
  bool dnNow = mcpRead(MCP_OCT_DN);
  if(upNow && !upPrev){ if(octaveShift<3){ octaveShift++; showOctaveLED(); } }
  if(dnNow && !dnPrev){ if(octaveShift>-3){ octaveShift--; showOctaveLED(); } }
  upPrev=upNow; dnPrev=dnNow;
}

/*** FN gyorsparancsok ***/
void handleFnShortcutByNote(uint8_t midiNote){
  // 1) MIDI csatorna: NOTE_BASE .. NOTE_BASE+15 → 1..16
  if(midiNote >= NOTE_BASE && midiNote <= NOTE_BASE+15){
    MIDI_CH = 1 + (midiNote - NOTE_BASE); // 1..16
    showOctaveLED();
    return;
  }
  // 2) Görbe: NOTE_BASE+16 .. NOTE_BASE+23 → 0..7
  if(midiNote >= NOTE_BASE+16 && midiNote <= NOTE_BASE+23){
    uint8_t cc = midiNote - (NOTE_BASE+16);
    if(cc<=7) currentCurve = (Curve)cc;
    return;
  }
  // 3) Utility: C4..G4
  switch(midiNote){
    case 60: // C4
      calib_locked = false; learn_count = 0;
      break;
    case 62: // D4
      calib_locked = true;
      break;
    case 64: // E4
      saveEEP();
      break;
    case 65: // F4
      resetEEP();
      dt_fast_us = 600.0f; dt_slow_us = 20000.0f;
      currentCurve = CURVE_LINEAR; calib_locked = true; fixedVelocity = 100;
      break;
    case 67: // G4
      printStatus();
      break;
    default: break;
  }
}
inline bool fnActive(){ return mcpRead(MCP_FN); }

/*** SysEx (USB-MIDI beolvasás) ***/
void handleSysExBuffer(uint8_t *buf, uint8_t len){
  // F0 7D <cmd> [data...] F7
  if(len < 4) return;
  if(buf[0] != 0xF0 || buf[len-1] != 0xF7) return;
  if(buf[1] != 0x7D) return; // non-commercial ID
  uint8_t cmd = buf[2];

  switch(cmd){
    case 0x01: { // curve select: F0 7D 01 cc F7  (cc=0..7)
      if(len<5) return;
      uint8_t curve = buf[3];
      if(curve <= 7) currentCurve = (Curve)curve;
    } break;
    case 0x02: { // calib start (unlock)
      calib_locked=false; learn_count=0;
    } break;
    case 0x03: { // calib lock
      calib_locked=true;
    } break;
    case 0x04: { // save EEPROM
      saveEEP();
    } break;
    case 0x05: { // factory default (EEPROM clear)
      resetEEP();
      dt_fast_us = 600.0f; dt_slow_us = 20000.0f;
      currentCurve = CURVE_LINEAR; calib_locked = true; fixedVelocity = 100;
    } break;
    case 0x06: { // ORGAN_FIXED velocity: F0 7D 06 vv F7 (1..127)
      if(len<5) return;
      uint8_t vv = buf[3]; if(vv<1) vv=1; if(vv>127) vv=127;
      fixedVelocity = vv;
    } break;
    case 0x07: { // MIDI channel: F0 7D 07 cc F7 (cc=1..16 hex byte)
      if(len<5) return;
      uint8_t cc = buf[3];
      if(cc>=1 && cc<=16) MIDI_CH = cc;
    } break;
    case 0x08: { // Program Change: F0 7D 08 pp F7 (0..127)
      if(len<5) return;
      uint8_t pp = buf[3];
      midiEventPacket_t p={0x0C,(uint8_t)(0xC0|(MIDI_CH-1)),pp,0};
      MidiUSB.sendMIDI(p);
    } break;
    case 0x0A: { // STATUS → kiírás Serial Monitorra
      printStatus();
    } break;
    default: break;
  }
}

void pollMIDIin(){
  static uint8_t sx[64];
  static uint8_t sxLen=0;
  midiEventPacket_t rx;
  while ((rx = MidiUSB.read()).header != 0){
    uint8_t cin = rx.header & 0x0F;
    switch(cin){
      case 0x04: // SysEx cont. (3B)
      case 0x07: // SysEx end  (3B)
      case 0x06: // SysEx end  (2B)
      case 0x05: // SysEx end  (1B)
      {
        uint8_t bytes[3]={rx.byte1,rx.byte2,rx.byte3};
        for(uint8_t i=0;i<3;i++){
          if(bytes[i]==0 && (cin==0x06 || cin==0x05)) break;
          if(sxLen<sizeof(sx)) sx[sxLen++]=bytes[i];
          if(bytes[i]==0xF7){ handleSysExBuffer(sx,sxLen); sxLen=0; break; }
        }
      } break;
      default: break;
    }
  }
}

/*** SETUP ***/
void setup(){
  pinMode(LED,OUTPUT); digitalWrite(LED,LOW);
  pinMode(KBD0,OUTPUT); pinMode(KBD1,OUTPUT); pinMode(KBD2,OUTPUT);
  pinMode(KBD3,OUTPUT); pinMode(KBD4,OUTPUT);
  for(uint8_t r=0;r<4;r++){ pinMode(KS_PINS[r],INPUT); pinMode(KF_PINS[r],INPUT); }

  digitalWrite(KBD3,LOW); digitalWrite(KBD4,LOW); // 0=2665 aktív, /G2B=LOW
  digitalWrite(KBD0,LOW); digitalWrite(KBD1,LOW); digitalWrite(KBD2,LOW);

  timer1_init();
  buildRev();
  Serial.begin(115200);

  if(loadEEP()){
    g_eep_loaded = true;
  }else{
    g_eep_loaded = false;
    dt_fast_us = 600.0f;
    dt_slow_us = 20000.0f;
    currentCurve = CURVE_LINEAR;
    calib_locked = true;
    fixedVelocity = 100;
  }

  // I2C + eszközök
  Wire.begin(); // D2/D3

  // MCP23X17 init @0x20
  if(!mcp.begin_I2C(0x20)){
    // ha nem látszik I²C-n, safe state
  }
  // GPA0..4 mint INPUT_PULLUP (Sustain, Extra, OctUP, OctDN, FN)
  for(uint8_t i=0;i<=4;i++){ mcp.pinMode(i, INPUT_PULLUP); }
  // GPB0..5 mint OUTPUT, alapból LOW (LED anódok)
  for(uint8_t i=8;i<=13;i++){ mcp.pinMode(i, OUTPUT); mcp.digitalWrite(i, LOW); }
  showOctaveLED();

  // ADS1115 init
  if(ads.begin(0x48)){ ads.setGain(GAIN_ONE); g_ads_ok=true; } else { g_ads_ok=false; }
}

/*** LOOP ***/
void loop(){
  pollMIDIin();
  handleButtons();
  processJoystick();

  bool any=false;

  for(uint8_t panel=0; panel<2; panel++){
    setEnableForPanel(panel);
    for(uint8_t y=0; y<8; y++){
      selCol(y);
      for(uint8_t r=0; r<4; r++){
        const bool vKS = rd(KS_PINS[r]);
        const bool vKF = rd(KF_PINS[r]);
        const int8_t iKS = REV_KS[panel][y][r];
        const int8_t iKF = REV_KF[panel][y][r];

        // ===== KS rising → NoteOn (blokkoló várakozás nélkül) =====
        if(iKS>=0){
          if(vKS && !lastKS[panel][y][r]){
            uint16_t tNow = tnow();
            tKS_ticks[iKS] = tNow;

            if(!noteOnSent[iKS]){
              if(tryComputeVelAndTrigger(iKS, tKS_ticks[iKS], tKF_ticks[iKS])){
                any=true;
              }
            }
          }
          lastKS[panel][y][r]=vKS;
        }

        // ===== KF rising → NoteOn (blokkoló várakozás nélkül) =====
        if(iKF>=0){
          if(vKF && !lastKF[panel][y][r]){
            uint16_t tNow = tnow();
            tKF_ticks[iKF] = tNow;

            if(!noteOnSent[iKF]){
              if(tryComputeVelAndTrigger(iKF, tKS_ticks[iKF], tKF_ticks[iKF])){
                any=true;
              }
            }
          }
          lastKF[panel][y][r]=vKF;
        }

        // ===== KÖZÖS NOTE OFF DÖNTÉS – mindkét kontakt alapján =====
        auto processOffFor = [&](int8_t idx, bool vks_now, bool vkf_now){
          if(idx < 0) return;

          if(!noteOnSent[idx]){ releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; return; }

          uint32_t nowMs = millis();
          if(nowMs - noteOnMs[idx] < MIN_NOTE_LEN_MS) return;

          const bool bothOpenNow = (!vks_now && !vkf_now);

          if(!releaseArm[idx] && (!vks_now || !vkf_now)){
            releaseArm[idx] = true;
            tFirstReleaseTicks[idx] = tnow();
          }

          static uint32_t bothOpenSinceMs[37]={0};

          if(bothOpenNow){
            if(bothOpenSinceMs[idx]==0) bothOpenSinceMs[idx]=nowMs;

            if(nowMs - bothOpenSinceMs[idx] >= RELEASE_DEBOUNCE_MS){
              uint8_t offVel = 0;
              if(USE_NOTE_OFF_VELOCITY && releaseArm[idx]){
                uint16_t t2 = tnow();
                int16_t dt_ticks = (int16_t)(t2 - tFirstReleaseTicks[idx]);
                if(dt_ticks < 0) dt_ticks = -dt_ticks;
                offVel = vel_from_dt_ticks((uint16_t)dt_ticks);
              }
              midiOffV((uint8_t)(NOTE_BASE+idx + 12*octaveShift), offVel);
              noteOnSent[idx]=false;
              ksDown[idx]=false;
              tKS_ticks[idx]=0; tKF_ticks[idx]=0;
              releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; bothOpenSinceMs[idx]=0;
              any=true;
            }
          }else{
            bothOpenSinceMs[idx]=0;
          }
        };

        processOffFor(iKS, vKS, vKF);
        processOffFor(iKF, vKS, vKF);
      }
    }
  }

  if(any){ MidiUSB.flush(); digitalWrite(LED,HIGH); delay(1); digitalWrite(LED,LOW); }
}
