// SPDX-License-Identifier: GPL-3.0-or-later
/*******************************************************
 * bORG v1.2 – Korg Modwave MKI -> Arduino Pro Micro USB-MIDI
 * 
 * Hardware (3.3 V only):
 *  - Matrix scan unchanged (KS/KF, 74HC138 on KBD0..4).
 *  - I²C bus on D2=SDA, D3=SCL.
 *  - MCP23017 (CJMCU-2317) @0x20:
 *      GPA0  Sustain (CC64) input, pull-up, active LOW
 *      GPA1  Extra pedal (CC67) input, pull-up, active LOW
 *      GPA2  Octave UP button input, pull-up, active LOW
 *      GPA3  Octave DOWN button input, pull-up, active LOW
 *      GPA4  FN button (reserved) input, pull-up, active LOW
 *      GPB0..2  RGB LED (DOWN) anodes via 220–330 Ω
 *      GPB3..5  RGB LED (UP)   anodes via 220–330 Ω
 *      LED cathodes -> GND (common cathode)
 *  - ADS1115 @0x48 (I²C):
 *      A0 <- Joystick VRx (Pitch Bend)
 *      A1 <- Joystick VRy (Mod Wheel / CC1)
 * 
 * SysEx (0x7D non-commercial):
 *   v1.0:
 *     F0 7D 01 cc F7        Select velocity curve (cc=0..7)
 *     F0 7D 02 F7           Calibration start (unlock)
 *     F0 7D 03 F7           Calibration lock (stop)
 *     F0 7D 04 F7           Save calibration + settings to EEPROM
 *     F0 7D 05 F7           Factory reset (defaults)
 *     F0 7D 06 vv F7        Set fixed velocity (1..127)
 *     F0 7D 0A F7           Print STATUS to Serial Monitor
 *   v1.1:
 *     F0 7D 07 cc F7        Set MIDI channel (cc=1..16, hex byte)
 *     F0 7D 08 pp F7        Program Change (pp=0..127)
 *   v1.2:
 *     Octave shift via buttons (−3..+3), LED color feedback:
 *       ±1 -> GREEN, ±2 -> YELLOW (R+G), ±3 -> RED, 0 -> off
 *     Joystick via ADS1115: PB center dead-zone, CC1 0..127
 *
 * License: GPL-3.0-or-later. KORG and KORG Modwave are trademarks of KORG Inc.
 *******************************************************/

#include <Arduino.h>
#include <MIDIUSB.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_ADS1X15.h>

// ====== EXISTING MATRIX PINS (from v1.0) ======
// KS0 moved to D14 (freeing D2/D3 for I²C)
const uint8_t KS_PINS[4] = {14, 5, 7, 9};     // KS0..KS3
const uint8_t KF_PINS[4] = {A0, A1, A2, A3};  // KF0..KF3

const uint8_t KBD0 = 4;   // A (LSB)
const uint8_t KBD1 = 6;   // B
const uint8_t KBD2 = 8;   // C (MSB)
const uint8_t KBD3 = 10;  // /G2A (panel select) – 0:2665, 1:2663
const uint8_t KBD4 = 16;  // /G2B (global enable) – LOW
const uint8_t LED  = 17;  // TX LED
const uint8_t PIN_FN = 15; // FN button (reserved), still local pin (optional)

// ====== I2C Devices ======
Adafruit_MCP23017 mcp;      // @0x20
Adafruit_ADS1115  ads;      // @0x48

// MCP pin mapping
// Inputs (GPA0..4):
const uint8_t MCP_SUSTAIN = 0; // GPA0
const uint8_t MCP_EXTRA   = 1; // GPA1
const uint8_t MCP_OCT_UP  = 2; // GPA2
const uint8_t MCP_OCT_DN  = 3; // GPA3
const uint8_t MCP_FN      = 4; // GPA4
// Outputs (GPB0..5):
const uint8_t MCP_DOWN_R = 0+8; // GPB0
const uint8_t MCP_DOWN_G = 1+8; // GPB1
const uint8_t MCP_DOWN_B = 2+8; // GPB2
const uint8_t MCP_UP_R   = 3+8; // GPB3
const uint8_t MCP_UP_G   = 4+8; // GPB4
const uint8_t MCP_UP_B   = 5+8; // GPB5

// ====== Scan / velocity config (unchanged) ======
const uint8_t  SAMPLES=1, MAJ=1;
const uint16_t SAMPLE_DELAY_US=0;
const uint16_t COL_SETTLE_US=100;
const uint16_t FOCUS_TIMEOUT_TICKS = 60000;
const uint8_t  FOCUS_POLL_US       = 3;

const uint8_t  RELEASE_DEBOUNCE_MS = 3;
const uint8_t  MIN_NOTE_LEN_MS     = 8;
const bool     USE_NOTE_OFF_VELOCITY = true;

uint8_t  MIDI_CH=1;
const uint8_t  MIN_VEL=1, MAX_VEL=127;
uint8_t  NOTE_BASE=48; // octave shift applies to this

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

// ====== NOTE MAPS (unchanged) ======
struct Contact { uint8_t panel,col,row; };
// ... (keep NOTE_KS / NOTE_KF / REV_KS / REV_KF from v1.0 as-is) ...

// ====== State (unchanged for scanning) ======
bool     ksDown[37]={0};
bool     lastKS[2][8][4] = {{{0}}};
bool     lastKF[2][8][4] = {{{0}}};
uint16_t tKS_ticks[37]={0}, tKF_ticks[37]={0};
bool     noteOnSent[37]={0};
uint32_t noteOnMs[37]={0};
bool     releaseArm[37]={0};
uint16_t tFirstReleaseTicks[37]={0};

// ====== Octave shift ======
int8_t octaveShift = 0; // −3..+3

// ====== Timer1 (unchanged) ======
static inline void timer1_init(){ TCCR1A=0; TCCR1B=_BV(CS11); TCNT1=0; }
static inline uint16_t tnow(){ return TCNT1; }

// ====== EEPROM persist (extend if needed later) ======
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
// keep saveEEP(), loadEEP(), resetEEP() from v1.0

// ====== MIDI helpers (Note: channel runtime variable) ======
static inline void midiOn(uint8_t note,uint8_t vel){
  midiEventPacket_t p={0x09,(uint8_t)(0x90|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p);
}
static inline void midiOffV(uint8_t note,uint8_t vel){
  midiEventPacket_t p={0x08,(uint8_t)(0x80|(MIDI_CH-1)),note,vel}; MidiUSB.sendMIDI(p);
}

// ====== Curves, calibration, velocity from dt (unchanged) ======
// keep applyCurve01(), updateCalib(), vel_from_dt_ticks() from v1.0

// ====== LED helpers via MCP ======
void mcpSetRGB(bool up, uint8_t r, uint8_t g, uint8_t b){
  // write each channel (active HIGH)
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
  // 0: off both
  mcpSetRGB(false,0,0,0); mcpSetRGB(true,0,0,0);
  if(octaveShift==0) return;
  bool up = octaveShift>0;
  uint8_t mag = abs(octaveShift); // 1..3
  uint8_t r=0,g=0,b=0;
  if(mag==1){ g=1; }
  else if(mag==2){ r=1; g=1; }   // yellow
  else { r=1; }                  // red
  mcpSetRGB(up, r,g,b);
}

// ====== ADS1115 joystick ======
bool g_ads_ok = false;
Adafruit_ADS1115 ads1115;

int16_t readADS(uint8_t ch){
  if(!g_ads_ok) return 0;
  return ads1115.readADC_SingleEnded(ch); // 0..32767
}

// Map ADS value to PB (±8191) and CC1 (0..127)
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
void sendPitchBend(int16_t bend){
  uint16_t u = (uint16_t)(bend + 8192);
  uint8_t lsb = u & 0x7F;
  uint8_t msb = (u>>7) & 0x7F;
  midiEventPacket_t p = {0x0E, (uint8_t)(0xE0 | (MIDI_CH-1)), lsb, msb};
  MidiUSB.sendMIDI(p);
}
void sendCC(uint8_t cc, uint8_t val){
  midiEventPacket_t p = {0x0B, (uint8_t)(0xB0 | (MIDI_CH-1)), cc, val};
  MidiUSB.sendMIDI(p);
}

void processJoystick(){
  if(!g_ads_ok){
    // fallback: neutral
    sendPitchBend(0);
    sendCC(1, 0);
    return;
  }
  int16_t vx = readADS(0); // VRx
  int16_t vy = readADS(1); // VRy

  // ADS1115 is signed 16b in lib, but single-ended returns 0..32767
  const int16_t MID = 16384;
  const int16_t DZ  = 800;    // deadzone ~2.4%
  const int16_t SPAN= 16384;  // positive span

  int16_t pb = mapPB(vx, MID, DZ, SPAN);
  uint8_t cc1 = mapCC(vy, 0, 32767);

  sendPitchBend(pb);
  sendCC(1, cc1);
}

// ====== MCP inputs ======
bool mcpRead(uint8_t pin){ return mcp.digitalRead(pin) == LOW; } // active LOW buttons/pedals

void handleButtons(){
  static uint32_t last=0;
  if(millis()-last < 10) return; // simple debounce
  last = millis();

  // Sustain CC64
  static bool sustPrev=false;
  bool sustNow = mcpRead(MCP_SUSTAIN);
  if(sustNow != sustPrev){
    sendCC(64, sustNow?127:0);
    sustPrev = sustNow;
  }

  // Extra CC67
  static bool extraPrev=false;
  bool extraNow = mcpRead(MCP_EXTRA);
  if(extraNow != extraPrev){
    sendCC(67, extraNow?127:0);
    extraPrev = extraNow;
  }

  // Octave shift
  static bool upPrev=false, dnPrev=false;
  bool upNow = mcpRead(MCP_OCT_UP);
  bool dnNow = mcpRead(MCP_OCT_DN);

  if(upNow && !upPrev){ if(octaveShift<3){ octaveShift++; showOctaveLED(); } }
  if(dnNow && !dnPrev){ if(octaveShift>-3){ octaveShift--; showOctaveLED(); } }

  upPrev=upNow; dnPrev=dnNow;
}

// ====== STATUS, SysEx, scan loop ======
// keep printStatus(), handleSysExBuffer(), pollMIDIin() from v1.1 (plus: on ch change call showOctaveLED() optionally)

// ====== Setup ======
void setup(){
  pinMode(LED,OUTPUT); digitalWrite(LED,LOW);
  pinMode(KBD0,OUTPUT); pinMode(KBD1,OUTPUT); pinMode(KBD2,OUTPUT);
  pinMode(KBD3,OUTPUT); pinMode(KBD4,OUTPUT);
  for(uint8_t r=0;r<4;r++){ pinMode(KS_PINS[r],INPUT); pinMode(KF_PINS[r],INPUT); }
  if(PIN_FN) { pinMode(PIN_FN, INPUT_PULLUP); }

  digitalWrite(KBD3,LOW); digitalWrite(KBD4,LOW);
  digitalWrite(KBD0,LOW); digitalWrite(KBD1,LOW); digitalWrite(KBD2,LOW);

  timer1_init();
  // buildRev(); // keep from v1.0
  Serial.begin(115200);

  Wire.begin(); // D2/D3
  // MCP init
  mcp.begin(0x20);
  // GPA0..4 inputs + pullups
  for(uint8_t i=0;i<=4;i++){ mcp.pinMode(i, INPUT); mcp.pullUp(i, HIGH); }
  // GPB0..5 outputs LOW
  for(uint8_t i=8;i<=13;i++){ mcp.pinMode(i, OUTPUT); mcp.digitalWrite(i, LOW); }
  showOctaveLED();

  // ADS1115 init
  if(ads1115.begin(0x48)){
    ads1115.setGain(GAIN_ONE); // ±4.096V full-scale -> 3.3V ok
    g_ads_ok = true;
  }else{
    g_ads_ok = false;
  }

  // EEPROM load or defaults (keep from v1.0)
  // if(loadEEP()) ... else defaults ...
}

// ====== Loop ======
void loop(){
  pollMIDIin();          // SysEx
  handleButtons();       // MCP inputs
  processJoystick();     // ADS1115 → PB & CC1

  // ---- matrix scanning block from v1.0 (unchanged) ----
  // setEnableForPanel(), selCol(), rd(), focusWaitCounterpartTicks(), processOffFor(), MidiUSB.flush() etc.
}
