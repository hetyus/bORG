# 🎹 bORG v1.2 (alpha)

**Project:** Korg Modwave MKI keyboard → Arduino Pro Micro (3.3 V) USB-MIDI controller  
**Status:** Hardware add-ons + firmware (MCP23017 + ADS1115 + FN shortcuts)  
**License:** GPL-3.0-or-later (`LICENSE`)

> KORG and KORG Modwave are trademarks of KORG Inc. This is an independent, community project (no warranty).

---

## What’s new vs v1.1

- **MCP23017 (CJMCU-2317) @0x20**: I/O expansion  
  – Inputs: Sustain (CC64), Extra (CC67), **Oct UP/DOWN**, **FN** (all pull-ups, active LOW)  
  – Outputs: two RGB LEDs (DOWN/UP) — anodes via **220–330 Ω**, common cathode to GND
- **ADS1115 @0x48**: joystick ADC  
  – `A0 ← VRx` → **Pitch Bend** (dead-zone around center)  
  – `A1 ← VRy` → **Mod Wheel** (CC1 0..127)
- **FN + Key shortcuts** (host-less config): MIDI channel, velocity curve, and utility actions on keys  
- **Octave shift** (−3..+3) with color feedback: ±1=GREEN, ±2=YELLOW (R+G), ±3=RED; 0 → off

---

## Hardware (3.3 V only)

- **MCU:** Arduino Pro Micro **3.3 V / 8 MHz** (ATmega32U4)
- **Original matrix** unchanged (from v1.0):
  - KS: `D14, D5, D7, D9`
  - KF: `A0, A1, A2, A3`
  - 74HC138 select: `D4, D6, D8` ; enables: `D10, D16`
- **I²C bus:** `D2=SDA`, `D3=SCL` (shared)
- **MCP23017 @0x20**  
  - Inputs (GPA0..4): `Sustain, Extra, OctUP, OctDN, FN` with internal pull-ups; active LOW  
  - Outputs (GPB0..5): `DOWN(R,G,B)`, `UP(R,G,B)` **anodes via 220–330 Ω**; cathodes to GND
- **ADS1115 @0x48**  
  - `A0 ← VRx` (PB), `A1 ← VRy` (CC1) — joystick powered from 3.3 V
- **I²C pull-ups:** typical modules have 10 k (`103`) on SDA/SCL; if missing, add **4.7 k** to 3.3 V.

If you prefer, use the detailed schematic in `docs/bORG_v1.2_detailed_schematic_EasyEDA.pdf`.  
(If the GitHub preview fails to load, download the raw file.)

---

## Software / Libraries

- **MIDIUSB** (Arduino)
- **Adafruit MCP23017 Arduino Library**
- **Adafruit ADS1X15** (ADS1115)

Board: SparkFun Pro Micro (3.3V, 8 MHz) or equivalent 32U4 core  
Serial Monitor: 115200 (STATUS prints)

---

## SysEx Command Reference (manufacturer ID 0x7D)

Format: `F0 7D <cmd> [data...] F7`

| Version | Cmd / Data           | Description                                | Example                          |
|--------:|----------------------|--------------------------------------------|----------------------------------|
| v1.0    | `01 cc (0..7)`       | Select velocity curve                      | `F0 7D 01 03 F7` → PIANO         |
|         | `02`                 | Calibration start (unlock)                  | `F0 7D 02 F7`                    |
|         | `03`                 | Calibration lock (stop)                     | `F0 7D 03 F7`                    |
|         | `04`                 | Save calibration + settings (EEPROM)        | `F0 7D 04 F7`                    |
|         | `05`                 | Factory reset                               | `F0 7D 05 F7`                    |
|         | `06 vv (1..127)`     | Set fixed velocity (ORGAN_FIXED)            | `F0 7D 06 64 F7` → 100           |
|         | `0A`                 | Print STATUS to Serial Monitor              | `F0 7D 0A F7`                    |
| v1.1    | `07 cc (1..16)`      | Set MIDI channel (hex byte)                 | `F0 7D 07 0A F7` → ch.10         |
|         | `08 pp (0..127)`     | Send Program Change                         | `F0 7D 08 14 F7` → PC#20         |
| v1.2.1  | —                    | Octave shift via buttons (−3..+3), LEDs     | —                                |

> Channel note: `cc` is one raw MIDI byte. Channel 16 = `0x10` (not decimal 16).

---

## FN + Key Shortcuts (v1.2)

Hold **FN** (GPA4, active LOW). While held, **keys trigger config** instead of notes.

- **MIDI channel**: the lowest **16** keys from `NOTE_BASE` (default C3) → channels 1..16  
- **Velocity curve**: the next **8** keys → `0..7 = LINEAR, FLAT, STEEP, PIANO, SYNTH, ORGAN_FIXED, SOFT, HARD`  
- **Utility (absolute notes):**  
  - **C4** → Calibration **START** (unlock)  
  - **D4** → Calibration **STOP** (lock)  
  - **E4** → **SAVE** (EEPROM)  
  - **F4** → **FACTORY RESET**  
  - **G4** → **STATUS** print to Serial

> LED feedback: octave LEDs show the current shift; saves/resets may blink briefly.

---

## Joystick (ADS1115)

- VRx (`A0`) → Pitch Bend (−8191..+8191), with small center dead-zone  
- VRy (`A1`) → Mod Wheel (CC1 0..127)  
- If ADS1115 is not detected, firmware falls back to PB=0, CC1=0 (safe).

---

## Bring-up / Test

1. Run an **I²C Scanner** — expect `0x20` (MCP23017) and `0x48` (ADS1115).  
2. Sustain/Extra toggle CC64/CC67 0↔127 (active LOW).  
3. Octave LEDs match shift (−3..+3) and color.  
4. Joystick PB centers; CC1 sweeps 0..127.  
5. FN shortcuts perform the intended config actions.

---

## Notes & Legal

- Changing MIDI channel does **not** send All Notes Off automatically in this build (keep keys released when switching).  
- KORG and KORG Modwave are trademarks of KORG Inc. This project is independent.  
- © 2025 hetyus — **GPL-3.0-or-later**. No warranty. Use at your own risk.
