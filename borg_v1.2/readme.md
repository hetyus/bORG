# 🎹 bORG v1.2 (alpha)

**Project:** Korg Modwave MKI keyboard → Arduino Pro Micro (3.3 V) USB-MIDI controller  
**Status:** Hardware add-ons + firmware refactor (ADS1115 + MCP23017)  
**License:** GPL-3.0-or-later (see `LICENSE`)

> KORG and KORG Modwave are trademarks of KORG Inc. This is an independent, community project, not affiliated with KORG.  
> **No warranty**: the software and hardware notes are provided “as is”.

---

## What’s new in v1.2 (compared to v1.1)

- **I²C GPIO expander:** MCP23017 (CJMCU-2317) at `0x20`  
  – Inputs (GPA0..4): Sustain (CC64), Extra (CC67), Octave **UP/DOWN**, FN (reserved)  
  – Outputs (GPB0..5): two **RGB LEDs** (DOWN/UP), each color via 220–330 Ω  
- **I²C ADC for joystick:** **ADS1115** at `0x48`  
  – `A0` ← VRx → **Pitch Bend** (with center dead-zone)  
  – `A1` ← VRy → **Mod Wheel** (CC1 0..127)  
- **Octave shift** (−3..+3) with LED color feedback:  
  – ±1 → **GREEN** · ±2 → **YELLOW** (R+G) · ±3 → **RED** · 0 → off  
- Codebase simplified (less boilerplate; matrix scan untouched)

---

## Repo layout
bORG/
├─ borg_v1.2/
│ ├─ bORG_v1.2.ino
│ ├─ readme.md ← (this file)
│ └─ docs/
│ ├─ bORG_v1.2_schematic_with_ADS1115.png
│ ├─ bORG_v1.2_wiring_overview.png
│ ├─ bORG_v1.2_pinmap.png
│ └─ bORG_v1.2_QuickReference_MCP23017.pdf
├─ borg_v1.1/ … (previous release)
├─ borg_v1.0/ … (baseline)
└─ LICENSE


---

## Hardware (3.3 V only)

- **MCU:** Arduino Pro Micro **3.3 V / 8 MHz** (ATmega32U4)
- **Key matrix** (unchanged from v1.0):
  - KS: `D14, D5, D7, D9`
  - KF: `A0, A1, A2, A3`
  - 74HC138 select: `D4, D6, D8` ; enables: `D10, D16`
- **I²C bus:** `D2=SDA`, `D3=SCL` (shared)
- **GPIO expander:** MCP23017 (CJMCU-2317) at `0x20`
  - **Inputs** (pull-ups, active LOW):  
    `GPA0=Sustain (CC64)`, `GPA1=Extra (CC67)`, `GPA2=Oct UP`, `GPA3=Oct DOWN`, `GPA4=FN`
  - **Outputs**:  
    `GPB0..2` → **DOWN** RGB anodes; `GPB3..5` → **UP** RGB anodes (each via 220–330 Ω).  
    LED cathodes → GND (common cathode)
- **ADC:** ADS1115 at `0x48`
  - `A0` ← Joystick **VRx** (Pitch Bend)
  - `A1` ← Joystick **VRy** (CC1)
- **Pedals/Buttons:** TIP → input, SLEEVE → GND (active LOW)
- **Power rails:** 3.3 V system; common GND
- **I²C pull-ups:** most modules have onboard 10 k (`103`). If missing, add **4.7 k** from SDA/SCL to 3.3 V.

**Diagrams:**  
- Schematic (MCP23017 + ADS1115): `docs/bORG_v1.2_schematic_with_ADS1115.png`  
- Overview / Pin map: `docs/bORG_v1.2_wiring_overview.png`, `docs/bORG_v1.2_pinmap.png`

---

## Software / Libraries

Install via Arduino Library Manager:

- **Adafruit MCP23017 Arduino Library**
- **Adafruit ADS1X15** (ADS1115/ADS1015)
- Core/board: Pro Micro 3.3 V/8 MHz (SparkFun or equivalent 32U4 core)

**Build tips**
- Tools → Board: SparkFun Pro Micro (3.3V, 8 MHz)
- Serial Monitor: 115200 baud (for status / I²C scanner)
- If you see I²C issues, first run the **I²C Scanner** (see below)

---

## SysEx Command Reference (manufacturer ID 0x7D)

Format: `F0 7D <cmd> [data...] F7`

| Version | Cmd / Data           | Description                                | Example                          |
|--------:|----------------------|--------------------------------------------|----------------------------------|
| v1.0    | `01 cc (0..7)`       | Select velocity curve                      | `F0 7D 01 03 F7` → PIANO         |
|         | `02`                 | Calibration start (unlock)                  | `F0 7D 02 F7`                    |
|         | `03`                 | Calibration lock (stop)                     | `F0 7D 03 F7`                    |
|         | `04`                 | Save calibration + settings (EEPROM)        | `F0 7D 04 F7`                    |
|         | `05`                 | Factory reset                               | `F0 7D 05 F7`                    |
|         | `06 vv (1..127)`     | Set fixed velocity (ORGAN_FIXED)            | `F0 7D 06 64 F7` → 100           |
|         | `0A`                 | Print STATUS to Serial Monitor              | `F0 7D 0A F7`                    |
| v1.1    | `07 cc (1..16)`      | Set MIDI channel (hex byte)                 | `F0 7D 07 0A F7` → ch.10         |
|         | `08 pp (0..127)`     | Send Program Change                         | `F0 7D 08 14 F7` → PC#20         |
| v1.2    | —                    | Octave shift via buttons (−3..+3), LEDs     | —                                |

> Note on channel: `cc` is a single data byte (hex). Channel 16 = `0x10` (not decimal 16).

---

## Octave LEDs (mapping)

- Shift = **0** → both LEDs off  
- **Negative** shift (−1..−3) → **DOWN** LED on  
- **Positive** shift (+1..+3) → **UP** LED on  
- Magnitude: `1 → GREEN`, `2 → YELLOW (R+G)`, `3 → RED`

---

## Joystick processing (ADS1115)

- Read **VRx** on `A0` → map to **Pitch Bend** (−8191..+8191)  
  – Center detection with small **dead-zone**  
- Read **VRy** on `A1` → map to **CC1** (0..127)

If the ADS1115 is not detected, firmware **falls back** to PB=0, CC1=0 (safe).

---

## I²C Scanner (bring-up)

Use this minimal sketch to verify the bus before first run:

```cpp
#include <Wire.h>
void setup(){ Wire.begin(); Serial.begin(115200); while(!Serial); }
void loop(){
  byte e,a; int n=0;
  for(a=1;a<127;a++){ Wire.beginTransmission(a); e=Wire.endTransmission();
    if(e==0){ Serial.print("I2C device at 0x");
      if(a<16) Serial.print("0"); Serial.println(a,HEX); n++; } }
  if(n==0) Serial.println("No I2C devices"); else Serial.println("done");
  delay(2000);
}
Expected:
0x20 → MCP23017 · 0x48 → ADS1115

##Troubleshooting

No I²C devices found: check SDA=D2, SCL=D3 wiring; common GND; 3.3 V power; pull-ups (103 on modules).

Joystick noisy: add dead-zone; ensure joystick VCC=3.3 V; keep analog leads short; consider simple RC (e.g. 1 k + 100 nF) if szükséges.

LED colors wrong: swap GPB wiring or anode order; verify common cathode type.

Sustain/Extra inverted: inputs are active LOW with pull-up; invert in wiring (TIP to input, SLEEVE to GND).

##Roadmap

v1.2.x: FN key mapping, config dump/load, small UX fixes

v1.3: OLED status, preset slots, optional DIN-MIDI out

Long-term: custom velocity curves via SysEx, ribbon/aftertouch (if hardware allows)

##Credits & Legal

© 2025 hetyus · GPL-3.0-or-later
KORG and KORG Modwave are trademarks of KORG Inc. This project is not affiliated with KORG.

> **No warranty**: the software and hardware notes are provided “as is”.
