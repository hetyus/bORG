# TODO – bORG v1.2.1

## Short term
- [ ] Retest velocity with parameters:  
  - MIN_DT_TICKS=380 (420 if needed)  
  - ARM_TIMEOUT_TICKS=60000  
  - COL_SETTLE_US=340  
  - INTER_SAMPLE_DELAY_US=50  
  - SAMPLES/MAJ=5/3  
- [ ] Compare single note vs chord behavior with/without STRICT_PAIRING  
- [ ] Verify ADS switch (USE_ADS=1) once ADS1115 wired  
- [ ] Check FN shortcuts and SysEx commands with I2C enabled

## Medium term
- [ ] Investigate and mitigate chord velocity spikes  
- [ ] Improve consistency of soft (pp) note detection  
- [ ] Add debug logging refinement (KS/KF pairing visualization)

## Long term
- [ ] Clean stable release tag for v1.2.1 (preset build)  
- [ ] Merge documentation: firmware variants under `/docs`  
- [ ] Prepare Quick Reference update (FN + SysEx table refresh)  
- [ ] Evaluate hardware consistency (switch tolerance, keybed differences)

---