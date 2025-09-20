# TODO – bORG v1.2.1 (USE_ADS minpatch)

## Immediate
- ✅ Confirm stable Note On/Off with `USE_ADS=0`.
- ⬜ Test build with `USE_I2C=1` + `USE_ADS=1` once ADS1115 arrives.
- ⬜ Verify FN shortcuts and SysEx handling remain unaffected by USE_ADS.

## Velocity
- ⬜ Investigate chord velocity spikes (multi-key presses).  
- ⬜ Tune `MIN_DT_TICKS` and `COL_SETTLE_US` for consistent results.  
- ⬜ Consider filtering black/white key imbalance (~30% higher velocity on black keys).  
- ⬜ Explore calibration routines for consistent velocity with repeated presses.

## Stability
- ⬜ Long-term test: sustain rapid key presses to ensure no missing Note On/Off.  
- ⬜ Verify release (Note Off) behavior, even if less critical.

## Documentation
- ⬜ Update repo docs: root README.md + per-version docs/README.md.  
- ⬜ Ensure Quick Reference matches actual FN/SysEx mapping.
