# 🎹 bORG v1.2 (Draft)

**Project:** Korg Modwave MKI keyboard → Arduino Pro Micro USB-MIDI controller  
**Planned Version:** 1.2  
**Status:** Development (work in progress)

---

## New Features in v1.2

### 1) Extended I/O via two PCF8574 (CJMCU-2317) expanders
- **Expander #1 (I²C 0x20)**: Sustain pedal, Extra pedal, Octave **UP/DOWN** buttons, **FN** button (optional).
- **Expander #2 (I²C 0x21)**: Two **common-cathode RGB LEDs** (DOWN/UP octave shift indicators).
- Inputs are **active-LOW** (internal pull-ups).  
- Each LED anode has a **220–330 Ω** series resistor. System runs at **3.3 V**.

### 2) Octave shift with LED feedback
- Buttons shift global transposition by **–3..+3 octaves** (relative to `NOTE_BASE`, default 48/C3).  
- LED code (only one LED lit at a time):  
  - **±1 octave → Green**  
  - **±2 octaves → Yellow** (R+G)  
  - **±3 octaves → Red**

### 3) Joystick for Pitch Bend / Mod Wheel
- 2-axis **10 kΩ** joystick: **A0 = Pitch**, **A1 = Mod** on the Pro Micro.  
- Pitch bend uses 14-bit range (–8192..+8191) with center dead-zone; Mod = CC1 (0..127).  
- Optional SW can go to an expander input.

---

## SysEx Command Reference

All SysEx messages use manufacturer ID **0x7D (non-commercial)**.  
Format: `F0 7D <command> [data...] F7`

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
| `07 cc` | `cc=1..16` | Set MIDI channel | `F0 7D 07 0A F7` → ch.10 |
| `08 pp` | `pp=0..127` | Send Program Change | `F0 7D 08 14 F7` → PC#20 |
| **1.2 (planned)** ||||
| *(TBD)* | – | Config dump/load presets | – |

---

## Hardware Overview

- Arduino **Pro Micro (3.3 V)**  
- 2 × **CJMCU-2317 / PCF8574** (0x20 inputs, 0x21 outputs)  
- 2 × RGB LED (common cathode) for octave shift  
- Sustain + extra pedal (mono jack, tip → input, sleeve → GND)  
- Octave UP/DOWN + FN push buttons  
- 2-axis joystick (A0/A1)

**Wiring diagrams:**  
- `docs/bORG_v1.2_Wiring.png`  
- `docs/bORG_v1.2_PCB_Placement.png`

---

## Backward Compatibility
- All v1.0 and v1.1 SysEx commands remain valid.  
- Note scanning + velocity logic unchanged.

---

## Roadmap
- v1.2.x: SysEx config dump/load, velocity layer presets.  
- v1.3: OLED display (curve/channel/octave).  
- Long-term: custom curves via SysEx, poly aftertouch/ribbon/joystick extensions, streamlined updates.

---

© 2025 hetyus.  
**KORG** and **KORG Modwave** are trademarks of KORG Inc. This project is not affiliated with or endorsed by KORG.  
Released under **GNU GPL v3.0** — see [LICENSE](../LICENSE).

