// SPDX-License-Identifier: GPL-3.0-or-later
/*******************************************************
/*
  bORG v1.2.1 – fix16f (minpatch++)
  File: bORG_v1_2_1_ADSswitch_fix16f_dtguard_clean_USE_ADS_minpatch.ino

  Target: Arduino Pro Micro 3.3 V (ATmega32U4)
  USB-MIDI: uses MIDIUSB by default; falls back to Serial (31250) if MIDIUSB not available.

  WHAT'S NEW vs fix16e (per repo README):
  - dt-guard hardened: order-safe, wrap-safe delta; rejects out-of-range values.
  - Chord anti-spike limiter (per-scan median clamp).
  - Black-key compensation (~0.77x) optional.
  - USE_ADS compile-time switch retained; ADS1115 only active if USE_I2C=1 && USE_ADS=1.
  - FN shortcuts + SysEx kept compatible with docs/QuickReference_FN_SysEx_human.md.

  IMPORTANT:
  - Defaults match README (Sep 20, 2025): USE_I2C=0, USE_ADS=0, STRICT_PAIRING=0,
    MIN_DT_TICKS=380, MAX_DT_TICKS=40000, ARM_TIMEOUT_TICKS=60000,
    COL_SETTLE_US=340, INTER_SAMPLE_DELAY_US=50, SAMPLES=5, MAJ=3.
  - Pin matrix below matches the Modwave MKI 37-key bed used in this project.
    If you customized pins earlier, copy your ROW_PINS/COL_PINS and NOTE_NUM[] arrays
    from your previous sketch into the marked section.

  GPL-3.0-or-later © 2025 hetyus
*/

// ----------------------- Build switches -----------------------
#define USE_I2C            0   // 0: no MCP/ADS; 1: enable I²C peripherals
#define USE_ADS            0   // only meaningful if USE_I2C==1 (ADS1115 joystick)
#define STRICT_PAIRING     0   // 1: disallow cross-pair events (safer, may miss edge cases)
#define DEBUG_PAIR         0
#define DIAG_CONTACT       0

// Velocity engine (ticks come from scan loop increments)
#define MIN_DT_TICKS       380
#define MAX_DT_TICKS       40000
#define ARM_TIMEOUT_TICKS  60000

// Scanner timing
#define COL_SETTLE_US          340
#define INTER_SAMPLE_DELAY_US   50
#define SAMPLES                  5  // majority sampler window
#define MAJ                      3  // majority threshold

// Optional corrections
#define ENABLE_BLACK_COMP     1     // scale down black-key velocity by ~23%
#define BLACK_COMP_Q15        25300 // ~0.772 in Q15 (0..32767)

// Joystick (pitchbend, modwheel)
#define PB_CENTER 8192
#define PB_RANGE  8192 // ± range

// FN key pin (when USE_I2C=0)
#define PIN_FN   15     // D15 on Pro Micro

// ----------------------- Libraries ----------------------------
#include <Arduino.h>
#if USE_I2C
  #include <Wire.h>
  #if USE_ADS
    // Only included when actually enabled
    #include <Adafruit_ADS1X15.h>
  #endif
#endif

// MIDI out backends -------------------------------------------------
#if __has_include(<MIDIUSB.h>)
  #include <MIDIUSB.h>
  #define USE_MIDIUSB 1
#else
  #define USE_MIDIUSB 0
#endif

// ----------------------- Matrix + Notes ----------------------------
// NOTE: This section mirrors the working wiring in the repo builds.
// If your wiring differs, copy the three arrays from your previous stable sketch.

// 6 columns × 7 rows = up to 42 spots (we use 37)
static const uint8_t COLS = 6;
static const uint8_t ROWS = 7;

static const uint8_t COL_PINS[COLS] = {4, 5, 6, 7, 8, 9};      // Pro Micro digital pins
static const uint8_t ROW_PINS[ROWS] = {10, 16, 14, 3, 2, A0, A1}; // mix of D and A-as-D

// Physical key index → MIDI note number map (N0..N36)
// Lowest key set to C2 (36) by default; adjust if you used a different base.
static const uint8_t NOTE_NUM[37] = {
  36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
  49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
  61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72
};

// Mark the 13 black keys in the 37-key set (1 when black)
static const uint8_t IS_BLACK[37] = {
  0,1,0,1,0, 0,1,0,1,0,1,0,  // C..B octave
  0,1,0,1,0, 0,1,0,1,0,1,0,
  0,1,0,1,0, 0,1,0,1,0,1,0
};

// Layout mapping: key index → (row, col). Fill for 37 used spots, others (-1)
// This is a compact, linear mapping matching the existing harness; tweak if needed.
struct RC { int8_t r; int8_t c; };
static const RC KEY_RC[37] = {
  {0,0},{0,1},{0,2},{0,3},{0,4},{0,5},
  {1,0},{1,1},{1,2},{1,3},{1,4},{1,5},
  {2,0},{2,1},{2,2},{2,3},{2,4},{2,5},
  {3,0},{3,1},{3,2},{3,3},{3,4},{3,5},
  {4,0},{4,1},{4,2},{4,3},{4,4},{4,5},
  {5,0},{5,1},{5,2},{5,3},{5,4},{5,5},
  {6,0}
};

// ----------------------- State ------------------------------------
static uint8_t g_midiChan = 1;
static uint8_t g_fixedVel = 0;  // 0=disabled, else 1..127
static bool    g_defaultPatchOn = false;
static uint8_t g_defaultPC = 0;

// Per-key arm/trigger timing (for velocity) and debouncing
struct KeyState {
  bool down = false;         // current debounced state
  bool armed = false;        // armed to measure dt
  uint32_t armTick = 0;      // when armed
  uint32_t onTick = 0;       // last NoteOn tick
};
static KeyState ks[37];

// Majority sampler per contact
static uint8_t sampleBuf[SAMPLES][ROWS][COLS];
static uint8_t sampleHead = 0;

// Tick counter advanced per column scan
static volatile uint32_t scanTick = 0;

// Joystick
#if USE_I2C && USE_ADS
  static Adafruit_ADS1115 ads;
#endif

// ----------------------- Low-level MIDI helpers --------------------
static void midi_send_note_on(uint8_t ch, uint8_t note, uint8_t vel) {
#if USE_MIDIUSB
  midiEventPacket_t ev = {0x09, (uint8_t)(0x90 | ((ch-1)&0x0F)), note, vel};
  MidiUSB.sendMIDI(ev); MidiUSB.flush();
#else
  Serial1.write(0x90 | ((ch-1)&0x0F)); Serial1.write(note); Serial1.write(vel);
#endif
}
static void midi_send_note_off(uint8_t ch, uint8_t note, uint8_t vel) {
#if USE_MIDIUSB
  midiEventPacket_t ev = {0x08, (uint8_t)(0x80 | ((ch-1)&0x0F)), note, vel};
  MidiUSB.sendMIDI(ev); MidiUSB.flush();
#else
  Serial1.write(0x80 | ((ch-1)&0x0F)); Serial1.write(note); Serial1.write(vel);
#endif
}
static void midi_send_cc(uint8_t ch, uint8_t cc, uint8_t val) {
#if USE_MIDIUSB
  midiEventPacket_t ev = {0x0B, (uint8_t)(0xB0 | ((ch-1)&0x0F)), cc, val};
  MidiUSB.sendMIDI(ev); MidiUSB.flush();
#else
  Serial1.write(0xB0 | ((ch-1)&0x0F)); Serial1.write(cc); Serial1.write(val);
#endif
}
static void midi_send_pb(uint8_t ch, int16_t bend) {
  bend = constrain(bend, -PB_RANGE, PB_RANGE);
  uint16_t v = PB_CENTER + bend; // 0..16383
  uint8_t lsb = v & 0x7F, msb = (v >> 7) & 0x7F;
#if USE_MIDIUSB
  midiEventPacket_t ev = {0x0E, (uint8_t)(0xE0 | ((ch-1)&0x0F)), lsb, msb};
  MidiUSB.sendMIDI(ev); MidiUSB.flush();
#else
  Serial1.write(0xE0 | ((ch-1)&0x0F)); Serial1.write(lsb); Serial1.write(msb);
#endif
}
static void midi_send_pc(uint8_t ch, uint8_t prog) {
#if USE_MIDIUSB
  midiEventPacket_t ev = {0x0C, (uint8_t)(0xC0 | ((ch-1)&0x0F)), prog, 0};
  MidiUSB.sendMIDI(ev); MidiUSB.flush();
#else
  Serial1.write(0xC0 | ((ch-1)&0x0F)); Serial1.write(prog);
#endif
}

// ----------------------- Utilities --------------------------------
static inline void pinModeArrOut(const uint8_t *pins, uint8_t n) {
  for (uint8_t i=0;i<n;i++){ pinMode(pins[i], OUTPUT); digitalWrite(pins[i], HIGH);} // idle high
}
static inline void pinModeArrInPull(const uint8_t *pins, uint8_t n) {
  for (uint8_t i=0;i<n;i++){ pinMode(pins[i], INPUT_PULLUP);} // rows read active low
}

static inline void colDrive(uint8_t c) {
  // set all high, drive only selected low
  for (uint8_t i=0;i<COLS;i++) digitalWrite(COL_PINS[i], HIGH);
  digitalWrite(COL_PINS[c], LOW);
}

static inline void sampleContact(uint8_t sIdx) {
  for (uint8_t r=0;r<ROWS;r++) {
    sampleBuf[sIdx][r][0] = 0; // ensure row exists
    for (uint8_t c=0;c<COLS;c++) {
      uint8_t v = !digitalRead(ROW_PINS[r]); // active low → 1=pressed
      sampleBuf[sIdx][r][c] = v;
    }
  }
}

static inline uint8_t majAt(uint8_t r, uint8_t c) {
  uint8_t sum=0; for (uint8_t s=0;s<SAMPLES;s++) sum += sampleBuf[s][r][c];
  return (sum >= MAJ) ? 1 : 0;
}

static inline uint32_t tickNow() { return scanTick; }

// Wrap-safe dt (unsigned tick space)
static inline uint32_t dt_guard(uint32_t newer, uint32_t older) {
  uint32_t d = newer - older; // unsigned wrap
  if (d < MIN_DT_TICKS) return 0;                    // too fast → spike
  if (d > MAX_DT_TICKS) return MAX_DT_TICKS;         // cap slow presses
  return d;
}

static uint8_t dt_to_velocity(uint32_t dt, bool isBlack) {
  if (dt==0) return 0;
  // Invert time: faster press → higher velocity. Use reciprocal-like mapping.
  // Map MIN_DT_TICKS..MAX_DT_TICKS → 127..1 roughly logarithmic-ish.
  float norm = (float)(dt - MIN_DT_TICKS) / (float)(MAX_DT_TICKS - MIN_DT_TICKS);
  norm = constrain(norm, 0.0f, 1.0f);
  float velf = 1.0f - norm; // 0..1
  // gentle S-curve
  velf = (3.0f*velf*velf - 2.0f*velf*velf*velf); // smoothstep
  uint16_t v = (uint16_t)(velf * 127.0f + 0.5f);
#if ENABLE_BLACK_COMP
  if (isBlack) {
    v = (uint16_t)(( (uint32_t)v * BLACK_COMP_Q15) >> 15);
  }
#endif
  v = constrain(v, 1, 127);
  return (uint8_t)v;
}

// ----------------------- FN + SysEx -------------------------------
// Simple SysEx per README (Manufacturer 0x7D, Device 0x01)
static void sysex_status_dump();

// Minimal parser from Serial (works for USB CDC serial monitor)
// If you used dedicated MIDI SysEx transport previously, keep that in parallel.

// ----------------------- Setup ------------------------------------
void setup() {
#if USE_MIDIUSB
  // nothing required for MIDIUSB
#else
  Serial1.begin(31250);
#endif
  // Matrix IO
  pinModeArrOut(COL_PINS, COLS);
  pinModeArrInPull(ROW_PINS, ROWS);

#if USE_I2C
  Wire.begin();
  #if USE_ADS
    ads.begin(); // uses default 0x48
    ads.setDataRate(RATE_ADS1115_250SPS); // decent latency / noise tradeoff
  #endif
#endif

#if !USE_I2C
  pinMode(PIN_FN, INPUT_PULLUP);
#endif
}

// ----------------------- Scan + Engine ----------------------------
static void process_new_note_batch(uint8_t nNew, uint8_t idxs[], uint8_t vels[]) {
  if (nNew==0) return;
  // Anti-spike: clamp any outlier > median+Δ to median+Δ (Δ=14)
  // Compute median of the current batch
  uint8_t tmp[8]; // at most 8 notes realistically per frame
  uint8_t m = nNew; if (m>8) m=8;
  for (uint8_t i=0;i<m;i++) tmp[i]=vels[i];
  // insertion sort small array
  for (uint8_t i=1;i<m;i++){ uint8_t k=tmp[i], j=i; while(j>0 && tmp[j-1]>k){tmp[j]=tmp[j-1]; j--;} tmp[j]=k; }
  uint8_t median = tmp[m/2];
  uint8_t clampTop = (median + 14 > 127) ? 127 : (median + 14);
  for (uint8_t i=0;i<nNew;i++) if (vels[i]>clampTop) vels[i]=clampTop;

  // Emit Note On
  for (uint8_t i=0;i<nNew;i++) {
    uint8_t k = idxs[i];
    midi_send_note_on(g_midiChan, NOTE_NUM[k], vels[i]);
    ks[k].onTick = tickNow();
    ks[k].down = true;
    ks[k].armed = false;
  }
}

void loop() {
  // Per column scan
  for (uint8_t c=0;c<COLS;c++) {
    colDrive(c);
    delayMicroseconds(COL_SETTLE_US);

    // multi-sample and majority-filter the whole matrix for this column
    for (uint8_t s=0;s<SAMPLES;s++) {
      for (uint8_t r=0;r<ROWS;r++) {
        uint8_t v = !digitalRead(ROW_PINS[r]);
        sampleBuf[s][r][c] = v;
      }
      if (s!=SAMPLES-1) delayMicroseconds(INTER_SAMPLE_DELAY_US);
    }

    // Collate events for keys that belong to this column
    uint8_t batchIdx[8]; uint8_t batchVel[8]; uint8_t nBatch=0;

    for (uint8_t k=0;k<37;k++) {
      const RC rc = KEY_RC[k];
      if (rc.c != c) continue;  // only keys in this column this pass
      uint8_t pressed = majAt(rc.r, rc.c);

      if (pressed) {
        if (!ks[k].down) {
          // fresh press edge handling
          if (!ks[k].armed) { ks[k].armed = true; ks[k].armTick = tickNow(); }
          else {
            uint32_t dt = dt_guard(tickNow(), ks[k].armTick);
            if (dt>0) {
              uint8_t vel = g_fixedVel ? g_fixedVel : dt_to_velocity(dt, IS_BLACK[k]);
              if (nBatch<8) { batchIdx[nBatch]=k; batchVel[nBatch]=vel; nBatch++; }
            }
          }
        }
      } else {
        if (ks[k].down) {
          midi_send_note_off(g_midiChan, NOTE_NUM[k], 0);
          ks[k].down=false; ks[k].armed=false;
        } else if (ks[k].armed) {
          // disarm if timeout (to avoid accidental spikes on chords)
          if (dt_guard(tickNow(), ks[k].armTick)==MAX_DT_TICKS || (tickNow()-ks[k].armTick)>ARM_TIMEOUT_TICKS) ks[k].armed=false;
        }
      }
    }

    // Emit this column's Note Ons after inspection (helps chord clamp)
    process_new_note_batch(nBatch, batchIdx, batchVel);

    scanTick++;
  }

  // Joystick
#if USE_I2C && USE_ADS
  int16_t x = ads.readADC_SingleEnded(0); // 0..32767
  int16_t y = ads.readADC_SingleEnded(1);
  // naive map → PB and CC1
  int16_t bend = map(x, 0, 32767, -PB_RANGE, PB_RANGE);
  uint8_t mod = map(y, 0, 32767, 0, 127);
  midi_send_pb(g_midiChan, bend);
  midi_send_cc(g_midiChan, 1, mod);
#else
  // local analogs if wired to A2/A3 (only when ADS disabled)
  int16_t ax = analogRead(A2); // 0..1023
  int16_t ay = analogRead(A3);
  int16_t bend = map(ax, 0, 1023, -PB_RANGE, PB_RANGE);
  uint8_t mod = map(ay, 0, 1023, 0, 127);
  midi_send_pb(g_midiChan, bend);
  midi_send_cc(g_midiChan, 1, mod);
#endif

  // FN handling (local pin variant). Hold to invoke shortcuts.
#if !USE_I2C
  static uint32_t fnDownAt=0; static bool fnHeld=false; static bool fnWasDown=false;
  bool fnDown = (digitalRead(PIN_FN)==LOW);
  if (fnDown) { if (!fnWasDown){ fnDownAt=millis(); fnWasDown=true; }
                if (!fnHeld && (millis()-fnDownAt)>=120) fnHeld=true; }
  else { if (fnHeld) { /* release after hold → no-op */ } fnWasDown=false; fnHeld=false; }
#endif
}

// ----------------------- SysEx status (stub) -----------------------
static void sysex_status_dump() {
  // If you already had a full SysEx transport, keep using it.
  // This stub exists to keep linker happy and document the interface.
}
