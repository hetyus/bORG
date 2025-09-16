# bORG v1.2.1 – Quick Reference

## FN Combos
- **CH 1..16**: FN + keys N0..N15
- **Status**: FN + N16 → print status
- **Factory reset**: FN + N17 → reset + defaults
- **Calibration**: FN + N18 start / FN + N19 lock
- **Save to EEPROM**: FN + N20
- **Fixed‑velocity mode**: FN + N21 (value via SysEx)
- **Curves**: FN + N22..N29 → `LINEAR, PIANO, SOFT, HARD, FLAT, STEEP, SYNTH, ORGAN_FIXED`
- **Default Patch ON/OFF**: FN + N26/N27 (if applicable)
- **Quick PC**: FN + N28..N36 → Program Change 0..8

> Adjust the exact key indices to match your mapping table in the sketch.

## SysEx (Manufacturer ID 0x7D)
| Cmd | Data        | Function                     | Example                      |
|-----|-------------|------------------------------|------------------------------|
| 01  | `cc` 0..7   | Velocity curve               | `F0 7D 01 03 F7` → PIANO     |
| 02  | –           | Calibration START            | `F0 7D 02 F7`                |
| 03  | –           | Calibration LOCK             | `F0 7D 03 F7`                |
| 04  | –           | Save to EEPROM               | `F0 7D 04 F7`                |
| 05  | –           | Factory reset                | `F0 7D 05 F7`                |
| 06  | `vv` 1..127 | Fixed velocity value         | `F0 7D 06 64 F7` → vel=100   |
| 07  | `ch` 1..16  | Global MIDI channel          | `F0 7D 07 0A F7` → CH10      |
| 08  | `pp` 0..127 | Send Program Change          | `F0 7D 08 14 F7` → PC#20     |
| 0A  | –           | Print STATUS over serial     | `F0 7D 0A F7`                |
