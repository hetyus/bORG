# 🎹 bORG 1.0

**Project:** Korg Modwave MKI keyboard → Arduino Pro Micro USB-MIDI controller  
**Version:** 1.0  
**Date:** 2025-09-09

---

## Release Notes

### Key Features
- **Velocity-sensitive keyboard scanning** – reads KS and KF contacts separately.  
- **SysEx configuration:**
  - 8 selectable velocity curves (LINEAR, FLAT, STEEP, PIANO, SYNTH, ORGAN_FIXED, SOFT, HARD).  
  - Fixed velocity in ORGAN_FIXED mode.  
  - Calibration (unlock → lock → save) to match your playing style.  
  - Factory reset command.  
  - STATUS command (prints current settings to Serial Monitor).  
- **NoteOff velocity support** – calculated from key release speed.  
- **Stable scanning & debounce** – reliable even with very fast or very slow playing.  
- **EEPROM storage** – calibration and curve selection persist across power cycles.  

### Technical Details
- **Hardware:** Arduino Pro Micro (ATmega32U4), Korg KLM-2663 + KLM-2665 keybed PCBs.  
- **MIDI:** USB-MIDI class compliant, works out-of-the-box with all major DAWs/OS.  
- **Range:** 37 keys (NOTE_BASE=48 / C3 → 84 / C6).  
- **Optimized I/O:** I²C bus freed for future expansion (pedals, LEDs, displays).  

---

## SysEx Command Reference

All SysEx messages use the **0x7D (non-commercial)** manufacturer ID.  
Format: `F0 7D <command> [data...] F7`

| Command | Description | Example |
|---------|-------------|---------|
| `01 cc` | Select velocity curve | `F0 7D 01 03 F7` → PIANO |
|         | 00=LINEAR, 01=FLAT, 02=STEEP, 03=PIANO, 04=SYNTH, 05=ORGAN_FIXED, 06=SOFT, 07=HARD | |
| `02` | Calibration start (unlock, learning mode) | `F0 7D 02 F7` |
| `03` | Calibration lock (stop learning) | `F0 7D 03 F7` |
| `04` | Save calibration + settings to EEPROM | `F0 7D 04 F7` |
| `05` | Factory reset (defaults: linear curve, locked calibration, fixedVel=100) | `F0 7D 05 F7` |
| `06 vv` | Set fixed velocity (ORGAN_FIXED mode) where vv=1..127 | `F0 7D 06 64 F7` → fixed velocity 100 |
| `0A` | Print STATUS to Serial Monitor | `F0 7D 0A F7` |

---

## Roadmap

### v1.1 (next minor release)
- SysEx control for **MIDI channel selection**.  
- SysEx-triggered **Program Change** option.  
- Improved LED feedback (e.g. longer blink when EEPROM is saved).  
- Separate **NoteOff velocity curves**.  

### v1.2
- **Extended I/O via I²C expanders** (e.g. CJMCU-2317 / PCF8574) for pedals and extra controls.  
- SysEx **dump/load of full configuration** (export/import presets).  
- **Velocity layer presets** (piano, organ, synth style ready-made configs).  

### Long-term goals
- **OLED display support** (curve, channel, preset visualization).  
- **Custom velocity curve editor via SysEx**.  
- Support for **polyphonic aftertouch, ribbon, joystick** (if hardware permits).  
- Streamlined **firmware update process**.  

---

## License
Non-commercial use, educational purposes.  
SysEx ID: **0x7D (non-commercial)**.
