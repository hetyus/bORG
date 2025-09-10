# bORG v1.0

**Release date:** 2025-09-10 (first public release)  
**Target hardware:** Arduino Pro Micro (ATmega32U4) + Korg Modwave MKI keybed

---

## Features in v1.0
- Velocity-sensitive key scanning  
- Multiple velocity curves (LINEAR, FLAT, STEEP, PIANO, SYNTH, ORGAN_FIXED, SOFT, HARD)  
- SysEx configuration:
  - Select curve (`F0 7D 01 cc F7`)
  - Calibration start/lock
  - Save to EEPROM
  - Factory reset
  - Set fixed velocity
  - STATUS printout  
- EEPROM storage for settings  
- NoteOff velocity support  

---

## Installation
1. Open `bORG_v1.0.ino` in Arduino IDE.  
2. Select **Arduino Pro Micro (ATmega32U4)** as the target board.  
3. Compile and upload.  

After upload, the device enumerates as a USB-MIDI controller.

---

## SysEx Reference (v1.0)
- **Velocity curve select:** `F0 7D 01 cc F7`  
- **Calibration start:** `F0 7D 02 F7`  
- **Calibration lock:** `F0 7D 03 F7`  
- **Save EEPROM:** `F0 7D 04 F7`  
- **Factory reset:** `F0 7D 05 F7`  
- **Set fixed velocity:** `F0 7D 06 vv F7`  
- **STATUS:** `F0 7D 0A F7`  

---

## Changelog
- **v1.0** — Initial release with velocity curves, calibration, EEPROM, SysEx config.

---

## License
GPL-3.0-or-later. See [LICENSE](../LICENSE).

© 2025 hetyus

---

## Trademarks
Korg® and Korg Modwave® are trademarks of their respective owner.  
This project is independent and unaffiliated with Korg Inc.
