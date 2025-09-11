# bORG v1.1

**Target hardware:** Arduino Pro Micro (ATmega32U4) + Korg Modwave MKI keybed

---

## New in v1.1
- SysEx:
  - Set MIDI channel: `F0 7D 07 cc F7` (cc=1..16)
  - Program Change: `F0 7D 08 pp F7` (pp=0..127)
  - Default patch enable+value: `F0 7D 09 ee pp F7` (ee=0/1, pp=0..127)
- FN + key configuration layer (no host required):
  - N0..N15 → MIDI channel 1..16
  - N16 STATUS, N17 FACTORY, N18 CALIB START, N19 CALIB LOCK, N20 SAVE
  - N21 FIXED-VEL, N22 LINEAR, N23 PIANO, N24 SOFT, N25 HARD
  - N26 DEFAULT PATCH OFF, N27 DEFAULT PATCH ON
  - N28..N36 → Program Change quick slots (0..8)
- Non-blocking LED feedback (save=long, status=double, channel=triple)
- Optional default patch sent ~300 ms after power-up

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
1. Open `borg_v1.1.ino` in Arduino IDE.  
2. Board: **Arduino Pro Micro (ATmega32U4)**.  
3. Upload.

**FN button wiring:** Pin 15 → switch → GND (INPUT_PULLUP, active LOW).

---

## Backward compatibility
- All v1.0 SysEx commands remain valid.
- Scanning/velocity logic unchanged.

---

## License
GPL-3.0-or-later. See [LICENSE](../LICENSE).  
**No warranty.** Provided *AS IS*.

© 2025 hetyus

---

## Trademarks
Korg® and Korg Modwave® are trademarks of their respective owner.  
This project is independent and unaffiliated with Korg Inc.
