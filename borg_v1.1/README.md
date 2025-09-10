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
