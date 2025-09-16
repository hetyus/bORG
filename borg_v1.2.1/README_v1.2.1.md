# bORG v1.2.1 (with ADS switch)

**bORG v1.2.1** turns a Korg Modwave MKI keybed into a USB‑MIDI controller using an Arduino Pro Micro 3.3 V plus:
- **MCP23017** for buttons/LEDs,
- **ADS1115** for joystick (Pitch Bend / Mod Wheel CC1).

This patch adds a build‑time **`USE_ADS` switch** to safely run without the ADC (no PB/CC1 spam).

## Hardware
- **MCU**: Pro Micro 3.3 V (ATmega32U4)
- **I²C**: SDA=D2, SCL=D3 @ 100 kHz
- **MCP23017 @ 0x20**: GPA0..4 inputs (sustain, extra, octave up/down, FN); GPB8..13 RGB LED outputs
- **ADS1115 @ 0x48**: A0=Joy X (Pitch Bend), A1=Joy Y (CC1)
- **FN button**: D15 when `USE_I2C=0`; on MCP when `USE_I2C=1`

## Build‑time flags
```c
#define USE_I2C 1   // 0=no MCP/ADS, 1=with MCP and optional ADS
#define USE_ADS 0   // 0=disable ADS code completely; 1=enable joystick
#define IGNORE_FN_FOR_TEST 0
```

## Timing / Sampling
```c
#define MIN_DT_TICKS    400   // velocity min delta (≈200 µs)
#define COL_SETTLE_US   150   // column settle
#define SAMPLES         3
#define MAJ             2
#define SAMPLE_DELAY_US 0
```

**Tuning tips**
- Dense chords → if velocity spikes (127): raise `COL_SETTLE_US` 150→200→260→320 µs.
- Early triggers → increase `MIN_DT_TICKS` 400→500→600.
- Only if needed: `SAMPLES=5` / `MAJ=3`.

## SysEx (0x7D manufacturer)
- `F0 7D 01 cc F7` – Velocity curve (0..7)
- `F0 7D 02 F7` – Calibration START
- `F0 7D 03 F7` – Calibration LOCK
- `F0 7D 04 F7` – Save EEPROM
- `F0 7D 05 F7` – Factory reset
- `F0 7D 06 vv F7` – Fixed velocity value (1..127)
- `F0 7D 07 ch F7` – Global MIDI channel (1..16)
- `F0 7D 08 pp F7` – Program Change (0..127)
- `F0 7D 0A F7` – Print STATUS

## FN shortcuts (summary)
- CH 1..16: FN + N0..N15
- Status: FN + N16
- Factory reset: FN + N17
- Calibration: FN + N18 (start) / N19 (lock)
- Save: FN + N20
- Fixed‑velocity mode: FN + N21 (set value via SysEx)
- Curves: FN + N22..N29 → `LINEAR, PIANO, SOFT, HARD, FLAT, STEEP, SYNTH, ORGAN_FIXED`
- Default Patch ON/OFF: FN + N26/N27 (if applicable)
- Quick PC: FN + N28..N36 → Program Change 0..8

## Running without ADS1115
If `USE_ADS=0` (recommended while ADS isn’t wired), joystick code is omitted and **no PB/CC1 messages** are sent.  
If `USE_ADS=1` but ADS is not present, the firmware silently does nothing (no spam).

## Known issues
- Rare velocity spikes on dense chords → increase `COL_SETTLE_US`, then `MIN_DT_TICKS`, then try `SAMPLES/MAJ`.
- First boot after flashing may show `EEPROM loaded: NO` → set params once and **Save**.

## Changelog (v1.2.1)
- Added `USE_ADS` switch and latching to eliminate ADS‑related MIDI spam
- Consolidated timing macros at top of sketch
- Preserved non‑blocking scan and velocity pipeline
