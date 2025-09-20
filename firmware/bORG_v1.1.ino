// SPDX-License-Identifier: GPL-3.0-or-later
// © 2025 hetyus

/*******************************************************
 * bORG v1.1.1 – Korg Modwave MKI → Arduino Pro Micro USB-MIDI
 *
 * *** SYSEX PARANCSOK ***
 * Gyártó ID: 0x7D (non-commercial)
 * Forma:  F0 7D <cmd> [adatok…] F7
 * (Minden adat NYERS MIDI BYTE 0..127, nem ASCII számjegy!)
 *
 * 0x01 – Velocity görbe választás
 *   F0 7D 01 cc F7
 *   cc: 00=LINEAR, 01=FLAT, 02=STEEP, 03=PIANO,
 *       04=SYNTH, 05=ORGAN_FIXED, 06=SOFT, 07=HARD
 *   Példa: F0 7D 01 03 F7 → PIANO
 *
 * 0x02 – Kalibráció START (unlock)
 *   F0 7D 02 F7
 *
 * 0x03 – Kalibráció LOCK
 *   F0 7D 03 F7
 *
 * 0x04 – Mentés EEPROM-ba
 *   F0 7D 04 F7    (LED: hosszú villan ~400 ms)
 *
 * 0x05 – Factory reset (alapbeállítások)
 *   F0 7D 05 F7
 *
 * 0x06 – Fix velocity (ORGAN_FIXED)
 *   F0 7D 06 vv F7   (vv = 1..127)
 *   Példa: F0 7D 06 64 F7 → fixedVel=100
 *
 * 0x07 – MIDI csatorna
 *   F0 7D 07 cc F7   (cc=1..16 → CH1..16)
 *   Példa: F0 7D 07 0A F7 → CH10
 *   (Firmware 0-bázist is elfogad: 0..15 → CH1..16)
 *
 * 0x08 – Program Change küldése
 *   F0 7D 08 pp F7   (pp=0..127)
 *   Példa: F0 7D 08 14 F7 → PC#20
 *
 * 0x09 – Default Patch engedély + érték
 *   F0 7D 09 ee pp F7   (ee=0/1, pp=0..127)
 *   Ha ON, boot után kiküldi a PC-t ~300 ms-nál.
 *
 * 0x0A – STATUS (soros monitorra)
 *   F0 7D 0A F7    (LED: dupla rövid)
 *
 * LED minták:
 *   SAVE/EEPROM: hosszú (~400 ms)
 *   STATUS: dupla rövid
 *   CH váltás: tripla gyors
 *
 * ------------------------------------------------------
 *
 * *** FN + BILLENTYŰ CONFIG RÉTEG (host nélkül) ***
 * FN = Arduino pin 15 (INPUT_PULLUP, aktív LOW)
 * Belépés: FN ≥120 ms → LED folyamatos
 * Kilépés: FN elengedés / 5 s tétlenség
 *
 * N0..N15   → MIDI CH1..CH16         (LED: tripla gyors)
 * N16       → STATUS                 (LED: dupla rövid)
 * N17       → FACTORY RESET          (LED: hosszú)
 * N18       → CALIB START            (LED: rövid)
 * N19       → CALIB LOCK             (LED: rövid)
 * N20       → SAVE EEPROM            (LED: hosszú)
 *
 * N21       → FIXED-VEL mód          (LED: rövid)
 * N22       → Görbe: LINEAR          (LED: rövid)
 * N23       → Görbe: PIANO           (LED: rövid)
 * N24       → Görbe: SOFT            (LED: rövid)
 * N25       → Görbe: HARD            (LED: rövid)
 *
 * N26       → Default Patch OFF      (LED: hosszú)
 * N27       → Default Patch ON       (LED: hosszú)
 *
 * N28..N36  → Gyors Program Change PC#0..8 (LED: rövid)
 *
 * Megjegyzések:
 * - Csatornaváltáskor All Notes Off az előző csatornán.
 * - Default Patch értékét SysEx 0x09 parancs állítja.
 * - LED visszajelzés nem zavarja a szkennelést.
 *******************************************************/



#include <Arduino.h>
#include <MIDIUSB.h>
#include <EEPROM.h>
#include <math.h>

/*** PINS – SERVICE MANUAL SZERINTI NEVEK ***/
// KS0 átrakva D3-ról D14-re (I2C felszabadítva: D2=SDA, D3=SCL)
const uint8_t KS_PINS[4] = {14, 5, 7, 9};     // KS0..KS3  (INPUT, külső 10k -> 3.3V)
const uint8_t KF_PINS[4] = {A0, A1, A2, A3};  // KF0..KF3  (INPUT, külső 10k -> 3.3V)

// 74HC138 cím- és enable-vonalak (KLM-2663/2665 rajz szerint)
const uint8_t KBD0 = 4;   // A (LSB)
const uint8_t KBD1 = 6;   // B (MID)
const uint8_t KBD2 = 8;   // C (MSB)
const uint8_t KBD3 = 10;  // /G2A (panel-váltó) – 0:2665, 1:2663
const uint8_t KBD4 = 16;  // /G2B (közös enable) – mindig LOW

const uint8_t LED  = 17;  // TX LED

/*** FUNKCIÓ gomb (v1.2-ben majd MCP23017) ***/
const uint8_t  FN_PIN = 15;               // aktív LOW
const uint16_t FN_HOLD_MS = 120;          // ennyi után lép be config módba
const uint16_t FN_IDLE_TIMEOUT_MS = 5000; // ennyi tétlenség után kilép

/*** SCAN / DEBOUNCE / FOCUS ***/
const uint8_t  SAMPLES=1, MAJ=1;          // ha zajos: 3/2
const uint16_t SAMPLE_DELAY_US=0;
const uint16_t COL_SETTLE_US=100;
const uint16_t FOCUS_TIMEOUT_TICKS = 60000; // Timer1 tick=0.5us → ~30 ms
const uint8_t  FOCUS_POLL_US       = 3;

/*** RELEASE / OFF kezelés ***/
const uint8_t  RELEASE_DEBOUNCE_MS = 3;   // „mindkettő nyitva” ennyi ideig
const uint8_t  MIN_NOTE_LEN_MS     = 8;   // minimum hanghossz
const bool     USE_NOTE_OFF_VELOCITY = true;

/*** MIDI (globális állapot) ***/
uint8_t  g_midiChannel = 1;      // 1..16
uint8_t  g_defaultPatch = 0;     // 0..127
bool     g_defaultPatchEnabled = false;
const uint8_t  MIN_VEL=1, MAX_VEL=127;
const uint8_t  NOTE_BASE=48;     // legalsó hang (C3)

/*** Görbék / Presetek ***/
enum Curve : uint8_t {
  CURVE_LINEAR=0, CURVE_FLAT=1, CURVE_STEEP=2, CURVE_PIANO=3,
  CURVE_SYNTH=4, CURVE_ORGAN_FIXED=5, CURVE_SOFT=6, CURVE_HARD=7
};
float   dt_fast_us = 600.0f;     // „gyors” dt (kiinduló)
float   dt_slow_us = 20000.0f;   // „lassú” dt
Curve   currentCurve = CURVE_LINEAR;
uint8_t fixedVelocity = 100;     // ORGAN_FIXED módban
bool    calib_locked = true;     // játék közben nem tanul
uint16_t learn_count = 0;

/*** NOTE MAP – (a te kiosztásod, KS==KF) ***/
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
uint16_t tKS_ticks[37]={0}, tKF_ticks[37]={0}; // Timer1 tick (0.5us)
bool     noteOnSent[37]={0};

// Config mód állapot
bool     configMode=false;           // FN nyomása után aktív
bool     fnPrev=false;               // előző mintavételi állapot
uint32_t fnDownSince=0;              // mikortól nyomva
uint32_t lastConfigActionMs=0;       // tétlenségi időzítő
bool     cfgLatched[37]={0};         // hogy egy billentyű csak egyszer süljön el config módban
const bool ENABLE_PC_SLOTS = true;   // ha nem kell a gyors PC, állítsd false-ra

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
  // 1.1 kiegészítések (a végére, back-compat)
  uint8_t  midiCh;       // 1..16
  uint8_t  defPatchEn;   // 0/1
  uint8_t  defPatch;     // 0..127
  uint8_t  _pad[2];
};
const uint32_t MAGIC = 0x4B4D5744UL; // 'KMWD'
const int EEPROM_ADDR = 0;

void saveEEP(){
  Persist p{};
  p.magic    = MAGIC;
  p.dt_fast  = dt_fast_us;
  p.dt_slow  = dt_slow_us;
  p.curve    = (uint8_t)currentCurve;
  p.locked   = calib_locked ? 1 : 0;
  p.fixedVel = fixedVelocity;
  p.midiCh   = g_midiChannel;
  p.defPatchEn = g_defaultPatchEnabled ? 1 : 0;
  p.defPatch = g_defaultPatch;
  EEPROM.put(EEPROM_ADDR, p);
}
bool loadEEP(){
  Persist p{}; EEPROM.get(EEPROM_ADDR, p);
  if(p.magic != MAGIC) return false;
  dt_fast_us    = p.dt_fast;
  dt_slow_us    = p.dt_slow;
  currentCurve  = (Curve)p.curve;
  calib_locked  = p.locked!=0;
  fixedVelocity = p.fixedVel ? p.fixedVel : 100;
  g_midiChannel = (p.midiCh >= 1 && p.midiCh <= 16) ? p.midiCh : 1;
  g_defaultPatchEnabled = (p.defPatchEn == 1);
  g_defaultPatch = (p.defPatch <= 127) ? p.defPatch : 0;
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
  uint8_t hit=0; for(uint8_t i=0;i<SAMPLES;i++){ if(digitalRead(pin)==LOW) hit++; if(SAMPLE_DELAY_US) delayMicroseconds(SAMPLE_DELAY_US); }
  return hit>=MAJ;
}

/*** MIDI segédek ***/
static inline uint8_t stNoteOn(){ return (uint8_t)(0x90 | ((g_midiChannel-1) & 0x0F)); }
static inline uint8_t stNoteOff(){ return (uint8_t)(0x80 | ((g_midiChannel-1) & 0x0F)); }
static inline uint8_t stProgCh (){ return (uint8_t)(0xC0 | ((g_midiChannel-1) & 0x0F)); }
static inline void midiOn(uint8_t note,uint8_t vel){ midiEventPacket_t p={0x09, stNoteOn(), note, vel}; MidiUSB.sendMIDI(p); }
static inline void midiOff(uint8_t note){ midiEventPacket_t p={0x08, stNoteOff(), note, 0};  MidiUSB.sendMIDI(p); }
static inline void midiOffV(uint8_t note,uint8_t vel){ midiEventPacket_t p={0x08, stNoteOff(), note, vel}; MidiUSB.sendMIDI(p); }
static inline void midiProgramChange(uint8_t program){ midiEventPacket_t p={0x0C, stProgCh(), program, 0}; MidiUSB.sendMIDI(p); }

/*** Nem-blokkoló LED állgép ***/
enum BlinkPattern { BLINK_NONE, BLINK_SHORT, BLINK_LONG, BLINK_DOUBLE_SHORT, BLINK_TRIPLE_QUICK };
struct BlinkState { BlinkPattern pat; uint8_t step; uint32_t t0; bool active; };
BlinkState g_led{BLINK_NONE,0,0,false};

static inline void ledSet(bool on){ digitalWrite(LED, on?HIGH:LOW); }
void ledBlink(uint8_t p){ g_led = { (BlinkPattern)p, 0, millis(), true }; }

void ledTask(){
  if(!g_led.active) return;
  const uint32_t now = millis();
  auto onOff = [&](uint16_t onMs, uint16_t offMs, uint8_t repeats){
    const uint32_t elapsed = now - g_led.t0;
    const uint32_t period = (uint32_t)onMs + offMs;
    const uint32_t idx = period? (elapsed / period) : 0;
    if(idx >= repeats){ g_led.active=false; ledSet(false); return; }
    const uint16_t phase = period? (elapsed % period) : 0;
    ledSet(phase < onMs);
  };
  switch(g_led.pat){
    case BLINK_SHORT:        onOff(50, 50, 1); break;
    case BLINK_LONG:         onOff(400, 0,  1); break;            // 400 ms on
    case BLINK_DOUBLE_SHORT: onOff(80, 80, 2); break;             // 2×(80/80)
    case BLINK_TRIPLE_QUICK: onOff(60, 60, 3); break;             // 3×(60/60)
    default: g_led.active=false; ledSet(false); break;
  }
}

/*** Görbék ***/
static inline float curve_piano(float x){
  const float k = 6.0f;                   // S-görbe meredekség
  return 1.0f / (1.0f + expf(-k*(0.5f - x))); // 0..1
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

/*** Kalibráció – csak külön módban aktív ***/
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

/*** Fókusz – a pár élének várása ugyanarra a cellára ***/
bool focusWaitCounterpartTicks(uint8_t panel, uint8_t col, uint8_t row, bool wantKS, uint16_t &tOutTicks){
  setEnableForPanel(panel);
  selCol(col);
  uint16_t t0 = tnow();
  while ((uint16_t)(tnow() - t0) < FOCUS_TIMEOUT_TICKS){
    bool v = rd(wantKS ? KS_PINS[row] : KF_PINS[row]);
    if(v){ tOutTicks = tnow(); return true; }
    if(FOCUS_POLL_US) delayMicroseconds(FOCUS_POLL_US);
  }
  return false;
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
  Serial.println(F("=== bORG v1.1.1 STATUS ==="));
  Serial.print(F("EEPROM loaded: ")); Serial.println(g_eep_loaded ? F("YES") : F("NO (defaults)"));
  Serial.print(F("Calib locked: ")); Serial.println(calib_locked ? F("YES") : F("NO (learning mode)"));
  Serial.print(F("dt_fast_us: ")); Serial.println(dt_fast_us, 1);
  Serial.print(F("dt_slow_us: ")); Serial.println(dt_slow_us, 1);
  Serial.print(F("Curve: ")); Serial.println(curveName(currentCurve));
  Serial.print(F("Fixed velocity (ORGAN_FIXED): ")); Serial.println(fixedVelocity);
  Serial.print(F("NOTE_BASE: ")); Serial.println(NOTE_BASE);
  Serial.print(F("MIDI Channel: ")); Serial.println(g_midiChannel);
  Serial.print(F("DefaultPatchEnabled: ")); Serial.println(g_defaultPatchEnabled ? F("YES") : F("NO"));
  Serial.print(F("DefaultPatch: ")); Serial.println(g_defaultPatch);
  Serial.print(F("ConfigMode: ")); Serial.println(configMode ? F("ON") : F("OFF"));
  Serial.println(F("======================"));
}

/*** Közös command-backend (SysEx és FN+Key ide hív) ***/
static inline void cmd_setChannel(uint8_t cc){
  if(cc>=1 && cc<=16){
    g_midiChannel=cc;
    Persist p{}; EEPROM.get(EEPROM_ADDR,p);
    if(p.magic==MAGIC){ p.midiCh=g_midiChannel; EEPROM.put(EEPROM_ADDR,p); }
    ledBlink(BLINK_TRIPLE_QUICK);
  }
}
static inline void cmd_save(){ saveEEP(); ledBlink(BLINK_LONG); }
static inline void cmd_factory(){
  resetEEP();
  dt_fast_us=600.0f; dt_slow_us=20000.0f;
  currentCurve=CURVE_LINEAR; calib_locked=true; fixedVelocity=100;
  g_midiChannel=1; g_defaultPatchEnabled=false; g_defaultPatch=0;
  ledBlink(BLINK_LONG);
}
static inline void cmd_status(){ printStatus(); ledBlink(BLINK_DOUBLE_SHORT); }
static inline void cmd_calibStart(){ calib_locked=false; learn_count=0; ledBlink(BLINK_SHORT); }
static inline void cmd_calibLock(){ calib_locked=true; ledBlink(BLINK_SHORT); }
static inline void cmd_setCurve(Curve c){ currentCurve=c; ledBlink(BLINK_SHORT); }
static inline void cmd_fixedVelMode(){ currentCurve=CURVE_ORGAN_FIXED; ledBlink(BLINK_SHORT); }
static inline void cmd_setDefaultPatch(bool en, uint8_t pp){
  if(pp<=127){ g_defaultPatchEnabled=en; g_defaultPatch=pp; saveEEP(); ledBlink(BLINK_LONG); }
}
static inline void cmd_sendPC(uint8_t pp){ if(pp<=127){ midiProgramChange(pp); MidiUSB.flush(); ledBlink(BLINK_SHORT);} }

/*** FN+Key kezelése ***/
void handleConfigKey(uint8_t noteIdx){
  lastConfigActionMs = millis();
  if(noteIdx <= 15){ cmd_setChannel(noteIdx+1); return; }
  switch(noteIdx){
    case 16: cmd_status();       return; // STATUS
    case 17: cmd_factory();      return; // FACTORY RESET
    case 18: cmd_calibStart();   return; // CALIB START
    case 19: cmd_calibLock();    return; // CALIB LOCK
    case 20: cmd_save();         return; // SAVE
    case 21: cmd_fixedVelMode(); return; // FIXED-VEL
    case 22: cmd_setCurve(CURVE_LINEAR); return;
    case 23: cmd_setCurve(CURVE_PIANO);  return;
    case 24: cmd_setCurve(CURVE_SOFT);   return;
    case 25: cmd_setCurve(CURVE_HARD);   return;
    case 26: cmd_setDefaultPatch(false, g_defaultPatch); return;
    case 27: cmd_setDefaultPatch(true,  g_defaultPatch); return;
    default: {
      if(ENABLE_PC_SLOTS && noteIdx>=28 && noteIdx<=36){ cmd_sendPC((uint8_t)(noteIdx-28)); return; }
    } break;
  }
}

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
      if(curve <= 7) cmd_setCurve((Curve)curve);
    } break;
    case 0x02: { cmd_calibStart(); } break;   // calib start (unlock)
    case 0x03: { cmd_calibLock(); } break;    // calib lock
    case 0x04: { cmd_save(); } break;         // save EEPROM
    case 0x05: { cmd_factory(); } break;      // factory default (EEPROM clear)
    case 0x06: { // ORGAN_FIXED velocity: F0 7D 06 vv F7 (1..127)
      if(len<5) return;
      uint8_t vv = buf[3]; if(vv<1) vv=1; if(vv>127) vv=127;
      fixedVelocity = vv; ledBlink(BLINK_SHORT);
    } break;
    case 0x0A: { cmd_status(); } break;       // STATUS → kiírás

    // 1.1 parancsok
    case 0x07: { if(len<5) return; uint8_t cc = buf[3]; cmd_setChannel(cc); } break;  // Set MIDI channel
    case 0x08: { if(len<5) return; uint8_t pp = buf[3]; cmd_sendPC(pp); } break;      // Program Change
    case 0x09: { if(len<6) return; uint8_t ee = buf[3]; uint8_t pp = buf[4]; cmd_setDefaultPatch(ee!=0, pp); } break; // Default patch
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

/*** FN gomb kezelése (nem-blokkoló) ***/
void fnTask(){
  bool fnNow = (digitalRead(FN_PIN)==LOW); // aktív LOW
  uint32_t now = millis();

  if(fnNow && !fnPrev){ fnDownSince = now; }
  if(fnNow && !configMode && (now - fnDownSince >= FN_HOLD_MS)){
    configMode = true; lastConfigActionMs = now; ledSet(true);           // állandó fény jelzi a módot
    for(uint8_t i=0;i<37;i++) cfgLatched[i]=false;                       // reset a latchinghez
  }
  if(!fnNow && configMode){ // elengedés → azonnal kilépünk
    configMode = false; ledSet(false);
    for(uint8_t i=0;i<37;i++) cfgLatched[i]=false;
  }
  // timeout
  if(configMode && (now - lastConfigActionMs >= FN_IDLE_TIMEOUT_MS)){
    configMode=false; ledSet(false);
    for(uint8_t i=0;i<37;i++) cfgLatched[i]=false;
  }
  fnPrev = fnNow;
}

/*** I²C bővítésekhez előkészített feladat (v1.2) ***/
void ioTask(){
  // MCP23017/ADS1115 kezelés ide kerül a következő verzióban.
}

/*** SETUP ***/
void setup(){
  pinMode(LED,OUTPUT); ledSet(false);
  pinMode(KBD0,OUTPUT); pinMode(KBD1,OUTPUT); pinMode(KBD2,OUTPUT);
  pinMode(KBD3,OUTPUT); pinMode(KBD4,OUTPUT);
  for(uint8_t r=0;r<4;r++){ pinMode(KS_PINS[r],INPUT); pinMode(KF_PINS[r],INPUT); }
  pinMode(FN_PIN, INPUT_PULLUP);

  digitalWrite(KBD3,LOW); digitalWrite(KBD4,LOW); // 0=2665 aktív, /G2B=LOW
  digitalWrite(KBD0,LOW); digitalWrite(KBD1,LOW); digitalWrite(KBD2,LOW);

  TCCR1A=0; TCCR1B=_BV(CS11); TCNT1=0; // timer1_init()

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
    g_midiChannel = 1;
    g_defaultPatchEnabled = false;
    g_defaultPatch = 0;
  }
}

/*** LOOP ***/
void loop(){
  // Boot Program Change (~300 ms után egyszer) – ha be van kapcsolva
  static bool bootPCdone=false; static uint32_t tBoot=millis();
  if(!bootPCdone && millis()-tBoot>=300){
    if(g_defaultPatchEnabled){ midiProgramChange(g_defaultPatch); MidiUSB.flush(); }
    bootPCdone=true;
  }

  pollMIDIin(); // SysEx kezelése
  fnTask();     // FUNKCIÓ gomb kezelése
  ioTask();     // (v1.2 stub)

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

        // ===== KS rising → NoteOn vagy Config akció =====
        if(iKS>=0){
          if(vKS && !lastKS[panel][y][r]){
            if(configMode){ if(!cfgLatched[iKS]){ handleConfigKey(iKS); cfgLatched[iKS]=true; } }
            else {
              uint16_t tNow = tnow();
              tKS_ticks[iKS] = tNow;
              if(tKF_ticks[iKS]==0 && !noteOnSent[iKS]){
                uint16_t t2; if(focusWaitCounterpartTicks(panel,y,r,false,t2)) tKF_ticks[iKS]=t2;
              }
              if(tKF_ticks[iKS]!=0 && !noteOnSent[iKS]){
                uint16_t dt = (uint16_t)(tKS_ticks[iKS] - tKF_ticks[iKS]);
                if((int16_t)dt < 0) dt = (uint16_t)(tKF_ticks[iKS] - tKS_ticks[iKS]);
                uint8_t vel = vel_from_dt_ticks(dt);
                midiOn(NOTE_BASE+iKS, vel); noteOnSent[iKS]=true; any=true;
                noteOnMs[iKS]=millis(); releaseArm[iKS]=false; tFirstReleaseTicks[iKS]=0;
                updateCalib((uint32_t)dt >> 1); // ha nincs lock
              }
            }
          }
          lastKS[panel][y][r]=vKS;
        }

        // ===== KF rising → NoteOn vagy Config akció =====
        if(iKF>=0){
          if(vKF && !lastKF[panel][y][r]){
            if(configMode){ if(!cfgLatched[iKF]){ handleConfigKey(iKF); cfgLatched[iKF]=true; } }
            else {
              uint16_t tNow = tnow();
              tKF_ticks[iKF] = tNow;
              if(tKS_ticks[iKF]==0 && !noteOnSent[iKF]){
                uint16_t t2; if(focusWaitCounterpartTicks(panel,y,r,true,t2)) tKS_ticks[iKF]=t2;
              }
              if(tKS_ticks[iKF]!=0 && !noteOnSent[iKF]){
                uint16_t dt = (uint16_t)(tKS_ticks[iKF] - tKF_ticks[iKF]);
                if((int16_t)dt < 0) dt = (uint16_t)(tKF_ticks[iKF] - tKS_ticks[iKF]);
                uint8_t vel = vel_from_dt_ticks(dt);
                midiOn(NOTE_BASE+iKF, vel); noteOnSent[iKF]=true; any=true;
                noteOnMs[iKF]=millis(); releaseArm[iKF]=false; tFirstReleaseTicks[iKF]=0;
                updateCalib((uint32_t)dt >> 1);
              }
            }
          }
          lastKF[panel][y][r]=vKF;
        }

        // ===== KÖZÖS NOTE OFF DÖNTÉS – mindkét kontakt alapján =====
        auto processOffFor = [&](int8_t idx, bool vks_now, bool vkf_now){
          if(idx < 0) return;

          // Config mód: ha mindkettő nyitott → engedjük a következő FN-latch-et
          if(configMode){ if(!vks_now && !vkf_now) cfgLatched[idx]=false; return; }

          // Ha nincs élő NoteOn, tisztítás és kilépés
          if(!noteOnSent[idx]){ releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; return; }

          // Minimum hanghossz
          uint32_t nowMs = millis();
          if(nowMs - noteOnMs[idx] < MIN_NOTE_LEN_MS) return;

          // Mindkettő nyitva?
          const bool bothOpenNow = (!vks_now && !vkf_now);

          // Első elengedés észlelése
          if(!releaseArm[idx] && (!vks_now || !vkf_now)){
            releaseArm[idx] = true;
            tFirstReleaseTicks[idx] = tnow();
          }

          // Debounce
          static uint32_t bothOpenSinceMs[37]={0};

          if(bothOpenNow){
            if(bothOpenSinceMs[idx]==0) bothOpenSinceMs[idx]=nowMs;

            if(nowMs - bothOpenSinceMs[idx] >= RELEASE_DEBOUNCE_MS){
              // Stabil OFF → NoteOff (opcionális velocity)
              uint8_t offVel = 0;
              if(USE_NOTE_OFF_VELOCITY && releaseArm[idx]){
                uint16_t t2 = tnow();
                int16_t dt_ticks = (int16_t)(t2 - tFirstReleaseTicks[idx]);
                if(dt_ticks < 0) dt_ticks = -dt_ticks;
                offVel = vel_from_dt_ticks((uint16_t)dt_ticks);
              }
              midiOffV(NOTE_BASE+idx, offVel);
              noteOnSent[idx]=false;
              ksDown[idx]=false;
              tKS_ticks[idx]=0; tKF_ticks[idx]=0;
              releaseArm[idx]=false; tFirstReleaseTicks[idx]=0; bothOpenSinceMs[idx]=0;
              any=true;
            }
          }else{
            // ha valamelyik visszazárt, elengedjük a debounce-ot
            bothOpenSinceMs[idx]=0;
          }
        };

        // Meghívjuk mindkét indexre az aktuális cella állapotával
        processOffFor(iKS, vKS, vKF);
        processOffFor(iKF, vKS, vKF);
      }
    }
  }

  if(any){ MidiUSB.flush(); ledBlink(BLINK_SHORT); }

  // Nem-blokkoló LED futtatása
  ledTask();
}

