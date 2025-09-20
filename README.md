# 🎹 bORG

bORG turns the KORG Modwave MKI keybed into a class-compliant USB-MIDI controller using an Arduino Pro Micro 3.3 V
It supports velocity curves, SysEx configuration, EEPROM save/load, and optional I2C peripherals (MCP23017, ADS1115).

Who the hell would need this anyway?
Well, it only makes sense if you’ve converted the Korg Modwave into a module and you’re left with a bare keybed that you couldn’t use without a "brain".

---

## Hardware
- **MCU**: Pro Micro 3.3 V (ATmega32U4)
- **I²C**: SDA=D2, SCL=D3 @ 100 kHz
- **MCP23017 @ 0x20**: GPA0..4 inputs (sustain, extra, octave up/down, FN); GPB8..13 RGB LED outputs
- **ADS1115 @ 0x48**: A0=Joy X (Pitch Bend), A1=Joy Y (CC1)
- **FN button**: D15 when `USE_I2C=0`; on MCP when `USE_I2C=1

---

## Quick links
- **Stable firmware (recommended):**  
  `/firmware/bORG_v1.2.1_ADSswitch_fix16f_preset_stable.ino`
- **Current development branch:**  
  `/firmware/bORG_v1.2.1_ADSswitch_fix16f_dtguard_adswitch.ino`
- **Documentation:** see `/docs` (fix16f README, Quick Reference, status template)
- **Releases:** see GitHub *Releases* section (e.g. v1.1.1)
- **License:** GPL-3.0-or-later (LICENSE in repo root)

---

## Features
- Velocity-sensitive key matrix scanning with selectable curves
- SysEx-based configuration (channel, curve, fixed velocity, factory reset, status)
- EEPROM save/load
- **v1.2.1**:  
  - *dt-guard*: order-safe, wrap-safe KS/KF delta, rejecting out-of-range values  
  - *STRICT_PAIRING*: optional cross-pair prevention  
  - *USE_ADS*: compile-time switch restored; ADS1115 only active if `USE_I2C=1 && USE_ADS=1`

---

## Repository layout
```
/firmware/           ← all .ino sketches (v1.1 and v1.2.1 variants)
/docs/               ← detailed documentation
CHANGELOG.md
STATUS_TEMPLATE_bORG_v1.2.1.txt
TODO.md
LICENSE
```

---

## Build instructions (v1.2.1 – fix16f)
**Target:** Arduino Pro Micro 3.3 V (ATmega32U4)  
**Peripherals (optional):** MCP23017 @0x20, ADS1115 @0x48

1. Open the `.ino` in Arduino IDE (board: *Arduino Leonardo/Micro* family).  
2. Adjust build switches at the top of the source:
   - `#define USE_I2C 0/1`
   - `#define USE_ADS 0/1` *(only meaningful if USE_I2C=1)*
   - `#define STRICT_PAIRING 0/1`
   - `#define DEBUG_PAIR 0/1`
   - `#define DIAG_CONTACT 0/1`
3. Recommended stable parameters (already set in preset build):
   - `MIN_DT_TICKS=380`, `MAX_DT_TICKS=40000`, `ARM_TIMEOUT_TICKS=60000`
   - `COL_SETTLE_US=340`, `INTER_SAMPLE_DELAY_US=50`
   - `SAMPLES=5`, `MAJ=3`
4. Upload → test single notes, dyads, triads, fast repetitions, pianissimo presses.

**Note:** Leave `USE_ADS=0` unless ADS1115 is actually wired.  

---

## SysEx implementation

ll SysEx messages use **Manufacturer ID 7D** (non-commercial), Device ID `01`.  
Format: `F0 7D <cmd> [data…] F7`

- **Set velocity curve**  
  `F0 7D 01 03 cc F7`  
  `cc = 00 LINEAR, 01 FLAT, 02 STEEP, 03 PIANO, 04 SYNTH, 05 ORGAN_FIXED, 06 SOFT, 07 HARD`

- **Start calibration**  
  `F0 7D 01 02 F7`  
  Enables calibration mode.  

- **Lock calibration**  
  `F0 7D 01 03 F7`  
  Stores calibration and returns to normal mode.  

- **Save to EEPROM**  
  `F0 7D 01 04 F7`  
  Persists all current settings.  

- **Factory reset**  
  `F0 7D 01 05 F7`  
  Restores defaults.  

- **Set fixed velocity**  
  `F0 7D 01 06 vv F7`  
  `vv = 1–127` (fixed value for all notes).  

- **Set MIDI channel**  
  `F0 7D 01 07 cc F7`  
  `cc = 1–16` (MIDI channel).  

- **Send Program Change**  
  `F0 7D 01 08 pp F7`  
  `pp = 0–127` (program number).  

- **Set default patch**  
  `F0 7D 01 09 ee pp F7`  
  `ee = 0/1` (disable/enable), `pp = 0–127` (program).  

- **Request status**  
  `F0 7D 01 0A F7`  
  Dumps current configuration over SysEx.

*All SysEx IDs use manufacturer ID `7D` (non-commercial).*

---

## FN shortcuts (with MCP23017 enabled)

When holding **FN** (≥120 ms), keys are re-mapped to configuration functions.  
The key index (`N0..N36`) refers to the physical position in the 37-key keybed (lowest key = N0).

- **N0–N15 → Set MIDI channel 1–16**  
  Each of the first 16 keys assigns the global MIDI channel.  
- **N16 → Status dump**  
  Sends a SysEx report with current configuration.  
- **N17 → Factory reset**  
  Restores default channel, curve, and settings.  
- **N18 → Calibration start**  
  Unlocks calibration mode for contact timing.  
- **N19 → Calibration lock**  
  Ends calibration and stores timing offset in RAM.  
- **N20 → Save to EEPROM**  
  Writes current settings permanently to EEPROM.  
- **N21 → Toggle fixed velocity**  
  Enables/disables fixed-velocity mode (value defined by SysEx).  
- **N22 → Velocity curve: LINEAR**  
- **N23 → Velocity curve: PIANO**  
- **N24 → Velocity curve: SOFT**  
- **N25 → Velocity curve: HARD**  
- **N26 → Default patch OFF**  
  Disables auto program change at startup.  
- **N27 → Default patch ON**  
  Enables auto program change at startup (last saved PC).  
- **N28–N36 → Quick Program Change slots (PC#0–8)**  
  Each key sends an immediate Program Change message.

*(On baseline builds without I2C, FN is handled via the local pin only.)*

---

## Version branches
- **v1.1**: legacy stable branch (see `/firmware`)  
- **v1.2.1**: current development branch (dt-guard, pairing, ADS switch)  

The SysEx and FN features remain consistent across branches.  

---

## License
This project is licensed under **GPL-3.0-or-later**.  
See the [LICENSE](LICENSE) file for details.

---

**No warranty.** This software is provided *AS IS*, without any warranty of any kind.

---

© 2025 hetyus

---

## Trademarks
Korg® and Korg Modwave® are trademarks of their respective owner.  
This project is independent and unaffiliated with Korg Inc.
