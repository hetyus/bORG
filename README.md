# 🎹 bORG

Korg Modwave MKI keybed → Arduino Pro Micro (ATmega32U4, 3.3 V) USB-MIDI firmware

This firmware turns the Modwave MKI keybed into a class-compliant USB-MIDI controller.  
It supports velocity curves, SysEx configuration, EEPROM save/load, and optional I2C peripherals (MCP23017, ADS1115).

Who the hell would need this anyway?
Well, it only makes sense if you’ve converted the Korg Modwave into a module and you’re left with a bare keybed that you couldn’t use without a "brain".

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

| Function | Example | Description |
|----------|---------|-------------|
| Set channel | `F0 7D 01 01 cc F7` | `cc` = MIDI channel (1–16) |
| Set curve | `F0 7D 01 02 cc F7` | `cc` = curve (0–6) |
| Set fixed velocity | `F0 7D 01 03 vv F7` | `vv` = 1–127 |
| Factory reset | `F0 7D 01 7E F7` | Reset to defaults |
| Request status | `F0 7D 01 7F F7` | Send current config as SysEx |

*All SysEx IDs use manufacturer ID `7D` (non-commercial).*

---

## FN shortcuts (with MCP23017 enabled)

- **FN + C**: Octave down  
- **FN + D**: Octave up  
- **FN + E**: Toggle fixed velocity  
- **FN + F**: Curve select  
- **FN + G**: Status dump (SysEx)  
- **FN + A**: Factory reset  

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
