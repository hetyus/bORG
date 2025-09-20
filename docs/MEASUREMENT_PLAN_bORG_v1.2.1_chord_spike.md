# Measurement Plan – bORG v1.2.1 (Chord Velocity Spikes)
Target sketch: `bORG_v1_2_1_ADSswitch_fix16e_dtguard_clean_USE_ADS_minpatch.ino`

## Test setup (baseline)
- Firmware: current *minpatch* (`USE_ADS` present; keep `USE_ADS=0` unless ADS1115 is wired).
- Start values:
  - `MIN_DT_TICKS=300`
  - `COL_SETTLE_US=340`
  - `INTER_SAMPLE_DELAY_US=50`
  - `SAMPLES/MAJ=5/3`
  - `STRICT_PAIRING=0`
- MIDI monitor: lightweight, shows timestamps & velocity.
- Physical: same USB port, stable supply, keyboard flat, stable room temperature.

## Logging (simple & actionable)
Add inside `tryComputeVelAndTrigger(...)`, right before `midiOn(...)`:
```cpp
Serial.print("TRIG i="); Serial.print(idx);
Serial.print(" ks=");    Serial.print(tKS_ticks[idx]);
Serial.print(" kf=");    Serial.print(tKF_ticks[idx]);
Serial.print(" dt=");    Serial.print(dt);
Serial.print(" vel=");   Serial.println(vel);
```
On each KS/KF edge (where `tKS_ticks[i]` / `tKF_ticks[i]` are set) add:
```cpp
Serial.print("EDGE type="); Serial.print(wantKS? "KS":"KF");
Serial.print(" i=");        Serial.print(i);
Serial.print(" t=");        Serial.println(tnow());
```

### Log format (CSV-friendly)
```
phase, chord_size, offset_ms, params, idx, ks, kf, dt, vel
TRIG, 2, 0, MIN=300 COL=340 ISD=50 S/M=5/3 SP=0, 12, 45780, 46110, 330, 89
EDGE, -, -, -, 12, 45780, -, -, -
EDGE, -, -, -, 12, -, 46110, -, -
```

## Test matrix (run in this order; 10 reps per case)
1) **Single note (control)** — Key: C only → expect tight spread (no random highs).
2) **Two-note chord** — Keys: C+E  
   - (a) Simultaneous press.  
   - (b) Staggered press ~10–15 ms (E after C) — confirm offset in logs by `EDGE` times.
3) **Three-note chord** — Keys: C+E+G  
   - (a) Simultaneous; (b) Staggered: C, then +10 ms E, +10 ms G.

## Parameter sweeps (one knob at a time)
A) **COL_SETTLE_US**: 260 → 300 → 340 → 380  
B) **MIN_DT_TICKS**: 300 → 360 → 420  
C) **SAMPLES/MAJ**: 3/2 → 5/3 → 7/4  
D) **STRICT_PAIRING**: 0 → 1 (with the best A–C)  
E) **INTER_SAMPLE_DELAY_US**: 30 → 50 → 70 (with best A–D)

## Acceptance / rejection criteria
- **Single note**: no >+20 spikes vs median. If 1/10 outlier → raise `MIN_DT_TICKS` one step.
- **Two-note chord (simult.)**: both velocities within ±15 of single-note median.  
  Reject if systematic 120–127 spike → raise `COL_SETTLE_US` or `INTER_SAMPLE_DELAY_US`.
- **Two-note chord (staggered)**: if spikes vanish vs simultaneous → KS/KF pairing ambiguity is root cause; consider `STRICT_PAIRING=1` if dropout rate acceptable.
- **Three-note chord**: similar to two-note; if only first/last spikes, compare `EDGE` times (KS of one crossing KF of another within a few 100 µs).

## Minimal change sequence (apply in this order)
1) Pick **best COL_SETTLE_US** (A).  
2) Tune **MIN_DT_TICKS** (B) to cut bogus ultra-fast dt.  
3) Lock **SAMPLES/MAJ** that doesn’t increase misses (C).  
4) Decide **STRICT_PAIRING** (D) by actual dropout rate.  
5) Fine-tune **INTER_SAMPLE_DELAY_US** (E) only if needed.

## Optional sanity checks
- **Black vs white keys**: run single-note test on a black key (e.g., C#). If ~30% higher persists, we’ll later add a per-row correction (not in this round).
- **USB flooding**: if your monitor freezes, retry with a lighter tool; keep Serial prints on a separate COM viewer.

## Deliverables (to iterate fast)
- One CSV (or pasted table) per parameter row with: median velocity per chord size, outlier count, chosen setting.  
- Short note: “Best combo so far = COL_SETTLE=xxx, MIN_DT=yyy, S/M=a/b, SP=z; chords OK/not OK”.

## Optional
- create macro-free patch that emits the CSV-style lines exactly as above and tags each run with the current parameters (so i can just copy-paste the Serial output)
