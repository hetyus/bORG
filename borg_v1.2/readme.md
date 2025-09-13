# 🎹 bORG v1.2 (alpha)

**Project:** Korg Modwave MKI keyboard → Arduino Pro Micro (3.3 V) USB-MIDI controller  
**Status:** Development (alpha)

---

## What’s new in v1.2

- **CJMCU-2317 (MCP23017) I/O expander** — 16 GPIO on a single I²C device (`0x20`):  
  - **Inputs (GPA0..4):** Sustain (CC64), Extra pedal (CC67), Octave **UP/DOWN**, **FN** (reserved).  
  - **Outputs (GPB0..5):** Two **RGB LEDs** (DOWN/UP) to indicate octave shift.  
- **Octave shift –3..+3** with LED feedback:  
  - Only one LED is lit: DOWN LED for negative, UP LED for positive.  
  - **±1 → Green; ±2 → Yellow (R+G); ±3 → Red; 0 → off.**  
- **Joystick** on **A4 = Pitch Bend** (14-bit, center dead-zone) and **A5 = Mod Wheel** (CC1).  
- v1.0/v1.1 SysEx **unchanged** (curves, calibration, EEPROM save, MIDI channel, Program Change).

---

## Hardware

- **MCU:** Arduino **Pro Micro (3.3 V)**  
- **I²C:** MCP23017 (CJMCU-2317) at **0x20**  
- **Joystick:** 10 kΩ, `VRx → A4`, `VRy → A5`, VCC=3.3 V, GND  
- **Pedals:** mono jacks; **TIP → input**, **SLEEVE → GND**; inputs are **active-LOW** (internal pull-ups).  
- **RGB LEDs:** **common cathode → GND**; each **anode → 220–330 Ω → MCP23017 output**.

### MCP23017 Pin Map (Adafruit library numbering)

- **Inputs (pull-ups enabled):**  
  - `GPA0 (pin 0)` = Sustain (CC64)  
  - `GPA1 (pin 1)` = Extra (CC67)  
  - `GPA2 (pin 2)` = Oct UP  
  - `GPA3 (pin 3)` = Oct DOWN  
  - `GPA4 (pin 4)` = FN (reserved)  
- **Outputs (LED anodes):**  
  - `GPB0 (pin 8)`  = DOWN R  
  - `GPB1 (pin 9)`  = DOWN G  
  - `GPB2 (pin 10)` = DOWN B  
  - `GPB3 (pin 11)` = UP R  
  - `GPB4 (pin 12)` = UP G  
  - `GPB5 (pin 13)` = UP B  

> If your board doesn’t have I²C pull-ups, add **4.7 kΩ** from **SDA** and **SCL** to **3.3 V**.

---

## Build

- **File:** `borg_v1.2/borg_v1.2.ino` (folder name must match the .ino filename).  
- **Board:** Arduino **Pro Micro (3.3 V)**  
- **Libraries:**  
  - **Adafruit MCP23017 Arduino Library** (`Adafruit_MCP23017.h`)  
  - Built-in: `Wire`, `MIDIUSB`, `EEPROM`

---

## SysEx Command Reference

All SysEx messages use manufacturer ID **0x7D (non-commercial)**.  
Format: `F0 7D <cmd> [data...] F7`

| Cmd | Data | Description | Example |
|-----|------|-------------|---------|
| `01 cc` | `cc=0..7` | Select velocity curve | `F0 7D 01 03 F7` → PIANO |
| `02` | – | Calibration start (unlock) | `F0 7D 02 F7` |
| `03` | – | Calibration lock (stop) | `F0 7D 03 F7` |
| `04` | – | Save calibration + settings to EEPROM | `F0 7D 04 F7` |
| `05` | – | Factory reset | `F0 7D 05 F7` |
| `06 vv` | `vv=1..127` | Set fixed velocity (ORGAN_FIXED) | `F0 7D 06 64 F7` → 100 |
| `0A` | – | Print STATUS to Serial Monitor | `F0 7D 0A F7` |
| **1.1** ||||
| `07 cc` | `cc=1..16` | Set MIDI channel | `F0 7D 07 0A F7` → channel 10 |
| `08 pp` | `pp=0..127` | Send Program Change | `F0 7D 08 14 F7` → Program 20 |
| **1.2** ||||
| – | – | Octave shift via buttons (–3..+3), LED feedback | – |

---

## Testing Checklist

- I²C works at **0x20**, inputs read as **LOW** when pressed/shorted.  
- Sustain (CC64) and Extra (CC67) toggle **0/127**.  
- Octave shift **–3..+3** changes note offset; LEDs match color policy.  
- Pitch Bend recenters (dead-zone), Mod Wheel (CC1) spans **0..127**.  
- SysEx 1.1: MIDI channel set, Program Change send OK.

---

## Notes, Trademarks, License

- Keep I²C runs short; twist **SDA/SCL** if possible.  
- **KORG** and **KORG Modwave** are trademarks of **KORG Inc.** This project is not affiliated with or endorsed by KORG.  
- © 2025 hetyus — Released under **GNU GPL v3.0 or later**. See [`LICENSE`](../LICENSE).
