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

## SysEx Commands

General format: `F0 7D <cmd> [data…] F7`  
– `7D` = non-commercial manufacturer ID  
– All values are **raw MIDI bytes (0–127)**, not ASCII characters!

| Cmd | Data | Function | Example |
|-----|------|----------|---------|
| `01 cc` | 00=LINEAR, 01=FLAT, 02=STEEP, 03=PIANO, 04=SYNTH, 05=ORGAN_FIXED, 06=SOFT, 07=HARD | Select velocity curve | `F0 7D 01 03 F7` → PIANO |
| `02` | – | Calibration START (unlock) | `F0 7D 02 F7` |
| `03` | – | Calibration LOCK | `F0 7D 03 F7` |
| `04` | – | Save settings to EEPROM | `F0 7D 04 F7` |
| `05` | – | Factory reset (defaults) | `F0 7D 05 F7` |
| `06 vv` | vv=1..127 | Set fixed velocity | `F0 7D 06 64 F7` → vel=100 |
| `07 cc` | cc=1..16 | Set global MIDI channel | `F0 7D 07 0A F7` → CH10 |
| `08 pp` | pp=0..127 | Send Program Change | `F0 7D 08 14 F7` → PC#20 |
| `09 ee pp` | ee=0/1, pp=0..127 | Enable/disable default patch at boot | `F0 7D 09 01 28 F7` → PC#40 on boot |
| `0A` | – | STATUS to Serial Monitor | `F0 7D 0A F7` |

**LED feedback patterns:**  
- EEPROM save → long blink (~400 ms)  
- STATUS → double short blink  
- MIDI channel change → triple quick blink  

---

## FN Button + Key Configuration (host-less control)

- FN button = Arduino pin 15 (INPUT_PULLUP, active LOW)  
- Enter config mode: hold FN ≥120 ms → LED solid ON  
- Exit: release FN or 5 s inactivity  

| Key (N index) | Function | LED |
|---------------|----------|-----|
| N0..N15  | MIDI channel 1..16 | Triple quick blink |
| N16      | STATUS | Double short blink |
| N17      | Factory reset | Long blink |
| N18      | Calibration START | Short blink |
| N19      | Calibration LOCK | Short blink |
| N20      | Save EEPROM | Long blink |
| N21      | Fixed-velocity mode | Short blink |
| N22      | Curve: LINEAR | Short blink |
| N23      | Curve: PIANO | Short blink |
| N24      | Curve: SOFT | Short blink |
| N25      | Curve: HARD | Short blink |
| N26      | Default Patch OFF | Long blink |
| N27      | Default Patch ON | Long blink |
| N28..N36 | Quick Program Change slots (PC#0..8) | Short blink |

---

## Notes

- When changing MIDI channel, the firmware sends an “All Notes Off” on the previous channel.  
- Default Patch value is configured with SysEx `0x09`.  
- LED feedback is non-blocking and does not interfere with key scanning.

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
