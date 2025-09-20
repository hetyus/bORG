# bORG v1.2.1 – dtguard + USE_ADS switch (minpatch)

## Overview
This firmware version is based on `bORG_v1.2.1_fix16e_dtguard_clean`, the last stable build with working Note On/Off.  
It introduces a minimal `USE_ADS` compile-time switch, allowing the ADS1115 joystick module to be enabled or disabled independently.

- Default: `USE_ADS=0` → ADS1115 is completely disabled.  
- `USE_ADS=1` requires `USE_I2C=1` and proper wiring of the ADS1115 module.

All other functionality remains identical to fix16e:  
matrix scanning, velocity computation, SysEx handling, and FN shortcuts.

---

## Compile-time parameters (at the top of the sketch)
```cpp
#define USE_I2C 0          // 0: I2C expander off, 1: on (MCP23017/ADS1115 connected)
#define USE_ADS 0          // 0: ADS1115 off, 1: on (requires USE_I2C=1)
#define IGNORE_FN_FOR_TEST 0
#define SHORT_FOCUS_WAIT_US 6000

#define MIN_DT_TICKS 300
#define COL_SETTLE_US 340
#define INTER_SAMPLE_DELAY_US 50
#define SAMPLES 5
#define MAJ 3
#define ARM_TIMEOUT_TICKS 60000
```

---

## FN Shortcuts
- **FN + Octave -** → Shift octave down
- **FN + Octave +** → Shift octave up
- **FN + C2** → Sustain toggle
- **FN + D2** → Curve selection
- **FN + E2** → Velocity mode
- **FN + F2** → Panic (All Notes Off)

---

## SysEx Commands
- `F0 7D 01 01 <value> F7` → Set curve
- `F0 7D 01 02 <value> F7` → Set velocity mode
- `F0 7D 01 7E F7` → Reset to defaults
- `F0 7D 01 7F F7` → Save to EEPROM

---

## Changelog
- **v1.2.1 fix16e (baseline)**: dtguard logic added, stable Note On/Off.  
- **v1.2.1 fix16e + USE_ADS (minpatch)**:  
  - Introduced `USE_ADS` flag.  
  - Guarded ADS includes, initialization, and `processJoystick()` body.  
  - No functional changes when `USE_ADS=0` (default).
