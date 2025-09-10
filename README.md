# 🎹 bORG v1.0

**Project:** Korg Modwave MKI keyboard → Arduino Pro Micro USB-MIDI controller

This firmware turns the Korg Modwave MKI keybed into a class-compliant USB-MIDI controller
with velocity curves, calibration, SysEx configuration, and EEPROM storage.

Who the hell would need this anyway?
Well, it only makes sense if you’ve converted the Korg Modwave into a module and you’re left with a bare keybed that you couldn’t use without a "brain".

---

## Features
- Velocity-sensitive scanning with selectable curves  
- SysEx configuration (curve, calibration, fixed velocity, factory reset, status)  
- NoteOff velocity support  
- EEPROM save/load of settings  

---

## Installation
1. Open the `.ino` file in Arduino IDE.  
2. Select **Arduino Pro Micro (ATmega32U4)** as board.  
3. Upload to your device.  

---

## Usage
Connect via USB. The device will show up as a MIDI controller.  
Configuration can be changed via SysEx messages (see documentation).

---

## Dependencies
- **Arduino Core** (LGPL)  
- **MIDIUSB library** (MIT)  

---

## License
This project is licensed under **GPL-3.0-or-later**.  
See the [LICENSE](LICENSE) file for details.

**No warranty.** This software is provided *AS IS*, without any warranty of any kind.

© 2025 hetyus

---

## Trademarks
Korg® and Korg Modwave® are trademarks of their respective owner.  
This project is independent and unaffiliated with Korg Inc.
