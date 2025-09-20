# bORG v1.2.1 – fix16f dt-guard + ADS switch

## Overview
Development variant with dt-guard logic, optional strict pairing, and restored ADS compile-time switch.  
Intended to reduce chord spikes and allow flexible testing of I2C/ADS configurations.

## Build switches
- `USE_I2C = 0/1`  
- `USE_ADS = 0/1` *(only relevant if I2C enabled)*  
- `STRICT_PAIRING = 0/1`  
- `DEBUG_PAIR = 0/1`  
- `DIAG_CONTACT = 0/1`

## Parameters
- `MIN_DT_TICKS = 380 (420 if needed)`  
- `MAX_DT_TICKS = 40000`  
- `ARM_TIMEOUT_TICKS = 60000`  
- `COL_SETTLE_US = 340`  
- `INTER_SAMPLE_DELAY_US = 50`  
- `SAMPLES / MAJ = 5 / 3`

## Known issues
- With `STRICT_PAIRING=0`: chord spikes still occur (rarer, not always 127).  
- With `STRICT_PAIRING=1`: soft note dropouts occur, especially on pianissimo presses.  

## Status
- Current focus: velocity stabilization (single vs. chord consistency).  
- dt-guard reduces spike frequency but not fully eliminated.  
- Ongoing parameter tuning required.  

## History
- Derived from fix16e, cleaned and stabilized.  
- Includes restored `USE_ADS` switch (removed in some intermediate builds).  
- Basis for the “preset stable” variant.
