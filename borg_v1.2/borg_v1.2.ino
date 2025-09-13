/*******************************************************
 * bORG v1.2 alpha – Korg Modwave MKI -> Arduino Pro Micro USB-MIDI
 * 3.3 V rendszer, 2x PCF8574 (CJMCU-2317), joystick, oktáv offset + LED visszajelzés
 *
 * ─────────────────────────────────────────────────────
 * HARDWARE OVERVIEW
 * - Arduino Pro Micro (3.3 V)
 * - PCF8574 #1 (I2C 0x20): INPUTS
 *     P0 = Sustain pedal (mono jack TIP)  → aktív LOW (SLEEVE → GND)
 *     P1 = Extra pedal (mono jack TIP)    → aktív LOW (SLEEVE → GND)  [CC67]
 *     P2 = Octave UP button               → aktív LOW
 *     P3 = Octave DOWN button             → aktív LOW
 *     P4..P7 = tartalék
 * - PCF8574 #2 (I2C 0x21): OUTPUTS (RGB LED-ek, common cathode)
 *     P0 = DOWN LED R (→ 220–330 Ω → LED anód)
 *     P1 = DOWN LED G (→ 220–330 Ω → LED anód)
 *     P2 = DOWN LED B (→ 220–330 Ω → LED anód)
 *     P3 = UP   LED R (→ 220–330 Ω → LED anód)
 *     P4 = UP   LED G (→ 220–330 Ω → LED anód)
 *     P5 = UP   LED B (→ 220–330 Ω → LED anód)
 *     P6..P7 = tartalék
 * - FN button: Arduino D15 (INPUT_PULLUP) – most még tartalék (hostless config későbbre)
 * - Joystick (10 kΩ, 2 tengely):
 *     VRx → A0 (Pitch Bend), VRy → A1 (Mod Wheel), VCC=3.3 V, GND=GND
 *
 * LED SZÍNKÓD (oktáv eltolás kijelzés, csak az egyik LED világít):
 *   0: mindkettő KI
 *  -1/+1: ZÖLD
 *  -2/+2: SÁRGA (R+G)
 *  -3/+3: PIROS
 *
 * ─────────────────────────────────────────────────────
 * SYSEX PARANCSOK (0x7D non-commercial ID)
 * Formátum: F0 7D <cmd> [adat...] F7
 *
 * v1.0:
 *   Görbe választás:           F0 7D 01 cc F7   (cc=0..7)
 *   Kalibráció START (unlock):  F0 7D 02 F7
 *   Kalibráció LOCK:            F0 7D 03 F7
 *   Mentés EEPROM:              F0 7D 04 F7
 *   Gyári alap (factory reset): F0 7D 05 F7
 *   Fix velocity (ORGAN_FIXED): F0 7D 06 vv F7  (vv=1..127)
 *   STATUS (Serial Monitor):    F0 7D 0A F7
 *
 * v1.1:
 *   MIDI csatorna:              F0 7D 07 cc F7  (cc=1..16)
 *   Program Change:             F0 7D 08 pp F7  (pp=0..127)
 *
 * v1.2 (tervezett jövő):
 *   Konfig dump/load:           TBA (későbbi minor)
 *
 * ─────────────────────────────────────────────────────
 * MEGJEGYZÉSEK
 * - 3.3 V rendszeren a LED soros ellenállás: 220–330 Ω javasolt.
 * - PCF8574 bemenetek: kvázi pull-up – a bitre '1'-et írunk, és úgy olvasunk.
 * - A szkennelést NEM zavarjuk: I2C polling időzített, rövid tranzakciókban fut.
 *
 * LICENSE
 * - GNU GPL v3.0
 * - KORG és KORG Modwave a KORG Inc. védjegyei. A projekt nem áll kapcsolatban a KORG-gal.
 *******************************************************/

#include <Arduino.h>
#include <MIDIUSB.h>
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>

/*** PINS – SERVICE MANUAL SZERINTI NEVEK ***/
// KS0 átrakva D3-ról D14-re (I2C felszabadítva: D2=SDA, D3=SCL)
const uint8_t KS_PINS[4] = {14, 5, 7, 9};     // KS0..KS3  (INPUT, külső 10k -> 3.3V)
const uint8_t KF_PINS[4] = {A0, A1, A2, A3};  // KF0..KF3  (INPUT, külső 10k -> 3.3V)  // MEGJ: v1.2-ben A0/A1 joystick! → Lásd NOTE alább

// 74HC138 cím- és enable-vonalak (KLM-2663/2665 rajz szerint)
const uint8_t KBD0 = 4;   // A (LSB)
const uint8_t KBD1 = 6;   // B (MID)
const uint8_t KBD2 = 8;   // C (MSB)
const uint8_t KBD3 = 10;  // /G2A (panel-váltó) – 0:2665, 1:2663
const uint8_t KBD4 = 16;  // /G2B (közös enable) – mindig LOW

const uint8_t LED  = 17;  // TX LED

/*** FONTOS v1.2 NOTE – ANALOG PINEK
 * Eredetileg KF_PINS[4] = A0..A3. v1.2-ben A0/A1-et joystick foglalja.
 * Ha a KF szenzor oldalon A0/A1 tényleg használatban volt, TÉNYLEGES HW-T ÉS KIOSZTÁST KELL IGAZÍTANI.
 * Az alábbi kódban a szkenner logikát érintetlenül hagyjuk (mint v1.0), feltételezve, hogy a KF oldali 4 bemenet
 * NEM ütközik a joystickkal (vagy át van kötve másik analógra).
 * Ha ütközés van: KF_PINS tömböt igazítsd olyan szabad analóg/lábakra, amelyek megfelelnek az eredeti elektronikának.
 */

/*** SCAN / DEBOUNCE / FOCUS ***/
const uint8_t  SAMPLES=1, MAJ=1;          // ha zajos: 3/2
const uint16_t SAMPLE_DELAY_US=0;
const uint16_t COL_SETTLE_US=100;
const uint16_t FOCUS_TIMEOUT_TICKS = 60000; // Timer1 tick=0.5us → ~30 ms
const uint8_t  FOCUS_POLL_US       = 3;

/*** RELEASE / OFF kezelés ***/
const uint8_t  RELEASE_DEBOUNCE_MS = 3;
const uint8_t  MIN_NOTE_LEN_MS     = 8;
const bool     USE_NOTE_OFF_VELOCITY = true;

/*** MIDI ***/
uint8_t  MIDI_CH=1;
const uint8_t  MIN_VEL=1, MAX_VEL=127;
const uint8_t  NOTE_BASE=48;              // legalsó hang (C3)

/*** Görbék / Presetek ***/
enum Curve : uint8_t {
  CURVE_LINEAR=0,
  CURVE_FLAT=1,
  CURVE_STEEP=2,
  CURVE_PIANO=3,
  CURVE_SYNTH=4,
  CURVE_ORGAN_FIXED=5,
  CURVE_SOFT=6,
  CURVE_HARD=7
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
  // v1.1+
  uint8_t  midiCh;     // 1..16
  // v1.2+
  int8_t   octShift;   // -3..+3
  uint8_t  _pad[3];
};
const uint32_t MAGIC = 0x4B4D5744UL;
const int EEPROM_ADDR = 0;

void saveEEP(){
  Persist p{};
  p.magic    = MAGIC;
  p.dt_fast  = dt_fast_us;
  p.dt_slow  = dt_slow_us;
  p.curve    = (uint8_t)currentCurve;
  p.locked   = calib_locked ? 1 : 0;
  p.fixedVel = fixedVelocity;
  p.midiCh   = MIDI_CH;
  p.octShift = 0; // futás közben is menthetjük, de default 0
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
  MIDI_CH       = (p.midiCh>=1 && p.midiCh<=16) ? p.midiCh : 1;
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
static inline void midiOn(uint8_t note,uint8_t vel){ midiEventPacket_t p={0x09,(uint8_t)(0x90|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p); }
static inline void midiOff(uint8_t note){ midiEventPacket_t p={0x08,(uint8_t)(0x80|(MIDI_CH-1)),note,0}; MidiUSB.sendMIDI(p); }
static inline void midiOffV(uint8_t note,uint8_t vel){ midiEventPacket_t p={0x08,(uint8_t)(0x80|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p); }
static inline void midiCC(uint8_t cc, uint8_t val){ midiEventPacket_t p={0x0B,(uint8_t)(0xB0|(MIDI_CH-1)),cc,val}; MidiUSB.sendMIDI(p); }
static inline void midiPB(int16_t bend){ // -8192..+8191
  uint16_t v = (uint16_t)(bend + 8192);
  uint8_t lsb = v & 0x7F;
  uint8_t msb = (v >> 7) & 0x7F;
  midiEventPacket_t p={0x0E,(uint8_t)(0xE0|(MIDI_CH-1)),lsb,msb};
  MidiUSB.sendMIDI(p);
}
static inline void midiPC(uint8_t prog){ midiEventPacket_t p={0x0C,(uint8_t)(0xC0|(MIDI_CH-1)),prog,0}; MidiUSB.sendMIDI(p); }

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
  Serial.println(F("=== bORG v1.2 STATUS ==="));
  Serial.print(F("EEPROM loaded: ")); Serial.println(g_eep_loaded ? F("YES") : F("NO (defaults)"));
  Serial.print(F("Calib locked: ")); Serial.println(calib_locked ? F("YES") : F("NO (learning mode)"));
  Serial.print(F("dt_fast_us: ")); Serial.println(dt_fast_us, 1);
  Serial.print(F("dt_slow_us: ")); Serial.println(dt_slow_us, 1);
  Serial.print(F("Curve: ")); Serial.println(curveName(currentCurve));
  Serial.print(F("Fixed velocity (ORGAN_FIXED): ")); Serial.println(fixedVelocity);
  Serial.print(F("NOTE_BASE: ")); Serial.println(NOTE_BASE);
  Serial.print(F("MIDI_CH: ")); Serial.println(MIDI_CH);
  Serial.print(F("Octave offset: ")); Serial.println((int)g_octaveOffset);
  Serial.println(F("======================"));
}

/*** v1.2 – OCTAVE OFFSET + LED ***/
int8_t g_octaveOffset = 0; // –3..+3

enum LedColor { LED_OFF=0, LED_GREEN, LED_YELLOW, LED_RED };

/*** PCF8574 – egyszerű driver ***/
const uint8_t PCF_IN_ADDR  = 0x20; // #1 inputs
const uint8_t PCF_OUT_ADDR = 0x21; // #2 outputs

// OUT regiszter cache – 8 bit (#2)
uint8_t g_pcfOut = 0x00;

// INPUT állapotok cache / előző
uint8_t g_pcfIn  = 0xFF; // default pull-up (1)
uint8_t g_pcfInPrev = 0xFF;

// I2C időzítés
uint32_t g_lastPcfPollMs = 0;
const uint8_t PCF_POLL_MS = 2; // rövid, nem blokkol

// Input bitkiosztás (#1)
const uint8_t IN_SUSTAIN = 0;   // P0
const uint8_t IN_EXTRA   = 1;   // P1
const uint8_t IN_OCT_UP  = 2;   // P2
const uint8_t IN_OCT_DN  = 3;   // P3
// P4..P7 reserve

// Output bitkiosztás (#2)
const uint8_t OUT_DOWN_R = 0;   // P0
const uint8_t OUT_DOWN_G = 1;   // P1
const uint8_t OUT_DOWN_B = 2;   // P2
const uint8_t OUT_UP_R   = 3;   // P3
const uint8_t OUT_UP_G   = 4;   // P4
const uint8_t OUT_UP_B   = 5;   // P5
// P6..P7 reserve

// Helper: I2C write 1 byte
bool pcfWrite8(uint8_t addr, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(val);
  return Wire.endTransmission()==0;
}
// Helper: I2C read 1 byte
bool pcfRead8(uint8_t addr, uint8_t &val){
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  val = Wire.read();
  return true;
}
// OUT maszk-bit beállítás
inline void pcfOutSet(uint8_t bit, bool on){
  if(on) g_pcfOut |=  (1<<bit);
  else   g_pcfOut &= ~(1<<bit);
}
// OUT LED-ek kiírása
inline void pcfFlushOut(){ pcfWrite8(PCF_OUT_ADDR, g_pcfOut); }

// INPUT-ok előkészítése (belső kvázi pull-up: 1-ek)
void pcfInit(){
  // Inputs expander: írjunk 1-eket → input/pullup
  pcfWrite8(PCF_IN_ADDR, 0xFF);
  // Outputs expander: nullázzuk
  g_pcfOut = 0x00;
  pcfFlushOut();
}

/*** LED logika – v1.2 ***/
void setDownLed(LedColor c){
  // DOWN: P0=R, P1=G, P2=B
  bool r=false,g=false,b=false;
  switch(c){
    case LED_OFF:    r=g=b=false; break;
    case LED_GREEN:  g=true;      break;
    case LED_YELLOW: r=true; g=true; break;
    case LED_RED:    r=true;      break;
  }
  pcfOutSet(OUT_DOWN_R, r);
  pcfOutSet(OUT_DOWN_G, g);
  pcfOutSet(OUT_DOWN_B, b);
}
void setUpLed(LedColor c){
  // UP: P3=R, P4=G, P5=B
  bool r=false,g=false,b=false;
  switch(c){
    case LED_OFF:    r=g=b=false; break;
    case LED_GREEN:  g=true;      break;
    case LED_YELLOW: r=true; g=true; break;
    case LED_RED:    r=true;      break;
  }
  pcfOutSet(OUT_UP_R, r);
  pcfOutSet(OUT_UP_G, g);
  pcfOutSet(OUT_UP_B, b);
}
void applyOctaveLeds(){
  if(g_octaveOffset==0){
    setDownLed(LED_OFF);
    setUpLed(LED_OFF);
  }else{
    LedColor c = (abs(g_octaveOffset)==1)?LED_GREEN : (abs(g_octaveOffset)==2)?LED_YELLOW : LED_RED;
    if(g_octaveOffset<0){ setDownLed(c); setUpLed(LED_OFF); }
    else                { setUpLed(c);   setDownLed(LED_OFF); }
  }
  pcfFlushOut();
}
inline int8_t clampOct(int8_t v){ if(v<-3) return -3; if(v>3) return 3; return v; }

/*** JOYSTICK – A0 (Pitch), A1 (Mod), 3.3 V; smoothing + dead-zone ***/
const uint8_t ADC_PITCH = A0;
const uint8_t ADC_MOD   = A1;

int readAdcFiltered(uint8_t pin){
  // 4-mintás egyszerű átlag
  uint16_t s=0; for(uint8_t i=0;i<4;i++){ s += analogRead(pin); }
  return (int)(s>>2);
}
// Pitch: 14-bit, dead-zone a közép körül
void processJoystick(){
  // ADC 10-bit feltételezve 0..1023 (Pro Micro)
  int vPitch = readAdcFiltered(ADC_PITCH);
  int vMod   = readAdcFiltered(ADC_MOD);

  // Pitch center normalize
  const int ADC_MID = 512;
  const int DZ = 20; // kb. ±2% dead-zone
  int delta = vPitch - ADC_MID;
  if(abs(delta) < DZ) delta = 0;

  // Skálázás -8192..+8191
  // 512-DZ → +8191; -512+DZ → -8192 (lineáris)
  float scale = (float)8191.0f / (512.0f - (float)DZ);
  int16_t bend = (int16_t)roundf(constrain(delta, -(512-DZ), (512-DZ)) * scale);
  if(bend != 0) midiPB(bend);

  // Mod CC1 0..127 (alapban lineáris)
  uint8_t modVal = (uint8_t)map(vMod, 0, 1023, 0, 127);
  midiCC(1, modVal);
}

/*** PCF INPUTOK – pedálok/gombok olvasása + eseménykezelés ***/
bool sustainOn=false, extraOn=false;

void handleInputs(){
  // Periodikus, rövid I2C
  uint32_t now=millis();
  if((uint8_t)(now - g_lastPcfPollMs) < PCF_POLL_MS) return;
  g_lastPcfPollMs = now;

  // read current
  uint8_t inVal=0xFF;
  if(pcfRead8(PCF_IN_ADDR, inVal)){
    g_pcfInPrev = g_pcfIn;
    g_pcfIn = inVal;
  }else{
    return; // I2C hiba – legközelebb
  }

  auto isLow = [&](uint8_t bit)->bool{ return (g_pcfIn & (1<<bit)) == 0; };
  auto wasLow= [&](uint8_t bit)->bool{ return (g_pcfInPrev & (1<<bit)) == 0; };

  // Sustain (CC64) – aktív LOW
  bool sustNow = isLow(IN_SUSTAIN);
  if(sustNow != sustainOn){
    sustainOn = sustNow;
    midiCC(64, sustainOn ? 127 : 0);
  }

  // Extra (CC67) – aktív LOW
  bool extraNow = isLow(IN_EXTRA);
  if(extraNow != extraOn){
    extraOn = extraNow;
    midiCC(67, extraOn ? 127 : 0);
  }

  // Octave UP – élérzékelés (HIGH->LOW)
  if(isLow(IN_OCT_UP) && !wasLow(IN_OCT_UP)){
    g_octaveOffset = clampOct(g_octaveOffset + 1);
    applyOctaveLeds();
  }
  // Octave DOWN – élérzékelés
  if(isLow(IN_OCT_DN) && !wasLow(IN_OCT_DN)){
    g_octaveOffset = clampOct(g_octaveOffset - 1);
    applyOctaveLeds();
  }
}

/*** SysEx (USB-MIDI beolvasás) – bővítve v1.1-gyel ***/
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
      digitalWrite(LED, HIGH); delay(30); digitalWrite(LED, LOW);
    } break;
    case 0x02: { // calib start (unlock)
      calib_locked=false; learn_count=0;
      digitalWrite(LED, HIGH); delay(80); digitalWrite(LED, LOW);
    } break;
    case 0x03: { // calib lock
      calib_locked=true;
      digitalWrite(LED, HIGH); delay(80); digitalWrite(LED, LOW);
    } break;
    case 0x04: { // save EEPROM
      saveEEP();
      digitalWrite(LED, HIGH); delay(400); digitalWrite(LED, LOW); // v1.1: hosszabb blink
    } break;
    case 0x05: { // factory default (EEPROM clear)
      resetEEP();
      dt_fast_us = 600.0f; dt_slow_us = 20000.0f;
      currentCurve = CURVE_LINEAR; calib_locked = true; fixedVelocity = 100;
      MIDI_CH=1; g_octaveOffset=0; applyOctaveLeds();
      digitalWrite(LED, HIGH); delay(200); digitalWrite(LED, LOW);
    } break;
    case 0x06: { // ORGAN_FIXED velocity: F0 7D 06 vv F7 (1..127)
      if(len<5) return;
      uint8_t vv = buf[3]; if(vv<1) vv=1; if(vv>127) vv=127;
      fixedVelocity = vv;
      digitalWrite(LED, HIGH); delay(50); digitalWrite(LED, LOW);
    } break;
    case 0x07: { // v1.1: MIDI channel set: F0 7D 07 cc F7  (cc=1..16)
      if(len<5) return;
      uint8_t cc = buf[3];
      if(cc>=1 && cc<=16) MIDI_CH = cc;
      // triple quick blink
      for(int i=0;i<3;i++){ digitalWrite(LED,HIGH); delay(40); digitalWrite(LED,LOW); delay(40); }
    } break;
    case 0x08: { // v1.1: Program Change: F0 7D 08 pp F7
      if(len<5) return;
      uint8_t pp = buf[3];
      midiPC(pp);
      digitalWrite(LED, HIGH); delay(40); digitalWrite(LED, LOW);
    } break;
    case 0x0A: { // STATUS → kiírás Serial Monitorra
      printStatus();
      // double short blink
      for(int i=0;i<2;i++){ digitalWrite(LED,HIGH); delay(30); digitalWrite(LED,LOW); delay(30); }
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
const uint8_t FN_PIN = 15; // D15 – INPUT_PULLUP (jövőre fenntartva)

void setup(){
  pinMode(LED,OUTPUT); digitalWrite(LED,LOW);
  pinMode(KBD0,OUTPUT); pinMode(KBD1,OUTPUT); pinMode(KBD2,OUTPUT);
  pinMode(KBD3,OUTPUT); pinMode(KBD4,OUTPUT);
  for(uint8_t r=0;r<4;r++){ pinMode(KS_PINS[r],INPUT); pinMode(KF_PINS[r],INPUT); }

  digitalWrite(KBD3,LOW); digitalWrite(KBD4,LOW); // 0=2665 aktív, /G2B=LOW
  digitalWrite(KBD0,LOW); digitalWrite(KBD1,LOW); digitalWrite(KBD2,LOW);

  pinMode(FN_PIN, INPUT_PULLUP);

  timer1_init();
  buildRev();
  Serial.begin(115200);

  // I2C init
  Wire.begin(); // Pro Micro 3.3 V – SDA=D2, SCL=D3
  pcfInit();

  if(loadEEP()){
    g_eep_loaded = true;
  }else{
    g_eep_loaded = false;
    dt_fast_us = 600.0f;
    dt_slow_us = 20000.0f;
    currentCurve = CURVE_LINEAR;
    calib_locked = true;
    fixedVelocity = 100;
    MIDI_CH = 1;
  }

  applyOctaveLeds(); // induló LED állapot
}

/*** LOOP ***/
void loop(){
  pollMIDIin(); // SysEx kezelése
  bool any=false;

  // --- BILLENTYŰ SZKENNELÉS (v1.0 eredeti logika) ---
  for(uint8_t panel=0; panel<2; panel++){
    setEnableForPanel(panel);
    for(uint8_t y=0; y<8; y++){
      selCol(y);
      for(uint8_t r=0; r<4; r++){
        const bool vKS = rd(KS_PINS[r]);
        const bool vKF = rd(KF_PINS[r]);
        const int8_t iKS = REV_KS[panel][y][r];
        const int8_t iKF = REV_KF[panel][y][r];

        // ===== KS rising → NoteOn =====
        if(iKS>=0){
          if(vKS && !lastKS[panel][y][r]){
            uint16_t tNow = tnow();
            tKS_ticks[iKS] = tNow;
            if(tKF_ticks[iKS]==0 && !noteOnSent[iKS]){
              uint16_t t2; if(focusWaitCounterpartTicks(panel,y,r,false,t2)) tKF_ticks[iKS]=t2;
            }
            if(tKF_ticks[iKS]!=0 && !noteOnSent[iKS]){
              uint16_t dt = (uint16_t)(tKS_ticks[iKS] - tKF_ticks[iKS]);
              if((int16_t)dt < 0) dt = (uint16_t)(tKF_ticks[iKS] - tKS_ticks[iKS]);
              uint8_t vel = vel_from_dt_ticks(dt);
              uint8_t note = (uint8_t)(NOTE_BASE+iKS + (int8_t)(12*g_octaveOffset)); // v1.2 transzponálás
              midiOn(note, vel); noteOnSent[iKS]=true; any=true;
              noteOnMs[iKS]=millis(); releaseArm[iKS]=false; tFirstReleaseTicks[iKS]=0;
              updateCalib((uint32_t)dt >> 1); // ha nincs lock
            }
          }
          lastKS[panel][y][r]=vKS;
        }

        // ===== KF rising → NoteOn =====
        if(iKF>=0){
          if(vKF && !lastKF[panel][y][r]){
            uint16_t tNow = tnow();
            tKF_ticks[iKF] = tNow;
            if(tKS_ticks[iKF]==0 && !noteOnSent[iKF]){
              uint16_t t2; if(focusWaitCounterpartTicks(panel,y,r,true,t2)) tKS_ticks[iKF]=t2;
            }
            if(tKS_ticks[iKF]!=0 && !noteOnSent[iKF]){
              uint16_t dt = (uint16_t)(tKS_ticks[iKF] - tKF_ticks[iKF]);
              if((int16_t)dt < 0) dt = (uint16_t)(tKF_ticks[iKF] - tKS_ticks[iKF]);
              uint8_t vel = vel_from_dt_ticks(dt);
              uint8_t note = (uint8_t)(NOTE_BASE+iKF + (int8_t)(12*g_octaveOffset)); // v1.2 transzponálás
              midiOn(note, vel); noteOnSent[iKF]=true; any=true;
              noteOnMs[iKF]=millis(); releaseArm[iKF]=false; tFirstReleaseTicks[iKF]=0;
              updateCalib((uint32_t)dt >> 1);
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
              uint8_t note = (uint8_t)(NOTE_BASE+idx + (int8_t)(12*g_octaveOffset)); // v1.2 transzponálás
              midiOffV(note, offVel);
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

  // --- I2C kontrollok és joystick feldolgozás rövid szakaszokban ---
  handleInputs();     // pedálok / octave gombok → CC + LED frissítés
  processJoystick();  // pitchbend / mod CC küldés

  if(any){ MidiUSB.flush(); digitalWrite(LED,HIGH); delay(1); digitalWrite(LED,LOW); }
}
