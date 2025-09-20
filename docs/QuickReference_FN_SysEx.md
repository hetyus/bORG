# Quick Reference – FN shortcuts & SysEx (human-readable)

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

---

## SysEx commands (Manufacturer ID 7D, Device ID 01)

All SysEx messages use **Manufacturer ID 7D** (non-commercial), Device ID `01`.  
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
