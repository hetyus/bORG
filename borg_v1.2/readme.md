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
