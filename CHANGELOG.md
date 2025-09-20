# Changelog

## v1.2.1_ADSswitch_fix16f_dtguard_adswitch (current development, dt-guard + ADS switch)
- dt‑guard (wrap/order safe).
- `USE_ADS` switch restored; all ADS1115 code under `(USE_I2C && USE_ADS)`.
- STRICT_PAIRING remains optional.

## v1.2.1-fix16e (dt-guard clean)
- dt‑guard (wrap/order safe).

## v1.2.1-fix13b (pairing only)
- STRICT_PAIRING introduced without dt-guard.
- Debug option for checking KS and KF idx

## v1.2.1-fix10 cleanhand
- clean, handmade(without regex automatization) sketch based on v1.2.1-I2Cswitch
- macros: COL_SETTLE_US, MIN_DT_TICKS, SAMPLES, MAJ, INTER_SAMPLE_DELAY_US, DROP_FIRST_SAMPLE_AFTER_COL
- known problems: inconsistant velocity, velocity spkikes when playing chords

## v1.2.1-fix1 to fix 9
- failed attempts to brind in a switch fo ADS, and failed attemtps on velocity optimalization
- Consolidated timing macros at top of sketch
- interpreting new macro INTER_SAMPLE_DELAY_US
- interpreting new macro DROP_FIRST_SAMPLE_AFTER_COL 1

## v1.2.1-I2Cswitch (baseline)
- Compile‑time I2C toggle
- Stable with I2C=0
- satarting velocity optimalization because of inconsistencies and spikes (MIN_DT_TICKS, COL_SETTLE_US, SAMPLES/MAJ)

## v1.2.1
- add FN knob -> FN + keys (hostless shortcuts):
 	– MIDI channel
        – Velocity curve:
        – Calib START, Calib STOP, SAVE, STATUS
- add MCP23017 for buttons/LEDs/pedals
   - Octave shift buttons (−3..+3), LED color feedback: ±1=GREEN, ±2=YELLOW (R+G), ±3=RED; 0 → LEDs off
   - Sustain pedal + SW pedal
- add ADS1115 for joystick (Pitch Bend / Mod Wheel CC1).
  * I²C: D2=SDA, D3=SCL
  * MCP23017 (Adafruit MCP23X17 lib) @0x20
  *   GPA0  Sustain (CC64)  INPUT_PULLUP, active LOW
  *   GPA1  Extra   (CC67)  INPUT_PULLUP, active LOW
  *   GPA2  Octave UP       INPUT_PULLUP, active LOW
  *   GPA3  Octave DOWN     INPUT_PULLUP, active LOW
  *   GPA4  FN button       INPUT_PULLUP, active LOW
  *   GPB0..2  RGB DOWN LED (R,G,B) anodes via 220–330Ω, cathode->GND
  *   GPB3..5  RGB UP   LED (R,G,B) anodes via 220–330Ω, cathode->GND
  *
  * ADS1115 @0x48:
  *   A0 <- Joystick X → Pitch Bend
  *   A1 <- Joystick Y → Mod Wheel (CC1)
- Consolidated timing macros at top of sketch
- Preserved non‑blocking scan and velocity pipeline

## v1.2alpha
- add FN knob -> FN + keys (hostless shortcuts):
- add MCP23017 for buttons/LEDs/pedals
- add ADS1115 for joystick (Pitch Bend / Mod Wheel CC1)
- pinout plan was bad -> rebuild in 1.2.1

## v1.1
- v.10 + new Sysex commands for midi channel change, program change, 

## v1.0 
- Initial release with velocity curves, calibration, EEPROM, SysEx config.
