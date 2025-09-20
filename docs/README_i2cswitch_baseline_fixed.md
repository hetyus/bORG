# bORG v1.2.1 – I2Cswitch baseline

## Overview
Clean baseline build, known to compile and run with I2C disabled.  
Provides a single compile-time switch to include/exclude both MCP23017 and ADS1115 logic.  
(ADS is always tied to I2C in this baseline, no separate USE_ADS flag yet.)

## Build switches
- `USE_I2C = 0/1` (if 0 → both MCP and ADS disabled)
- `IGNORE_FN_FOR_TEST = 0/1`
- `STRICT_PAIRING = 0/1`
- `DEBUG_PAIR = 0/1`
- `DIAG_CONTACT = 0/1`

## Parameters
- `MIN_DT_TICKS = 300–400`  
- `MAX_DT_TICKS = 40000`  
- `ARM_TIMEOUT_TICKS = (not critical, pairing disabled)`  
- `COL_SETTLE_US = 150` (baseline default)  
- `INTER_SAMPLE_DELAY_US = 30`  
- `SAMPLES / MAJ = 3 / 2`

## Known issues
- Chord spikes possible (KS/KF crossing not filtered).  
- No dt-guard in this baseline.  
- ADS logic included if I2C enabled, but not separately switchable.

## Status
- Stable for non-I2C builds.  
- Used as a reference for later ADS/dtguard variants.  

## History
- Derived from v1.2.1 development.  
- Serves as “clean baseline” for debugging and testing.
