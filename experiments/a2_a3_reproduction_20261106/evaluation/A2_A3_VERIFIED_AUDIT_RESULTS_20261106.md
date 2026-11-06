# A2 provenance and A3 early-window audit — verified console results (2026-11-06)

## Evidence and limits

These numbers are copied from the user's local terminal execution of `evaluation/a2_a3_offline_consistency_audit.py`. The original NPZ datasets remain local and have not been uploaded to this public repository. No new estimator run or independent reprocessing was performed here. "Position RMSE" denotes first-sample alignment, not metric-map/global coordinate absolute accuracy.

## Priority 1 — A2 provenance

- A2 PhysX truth: 241 samples, 0.000–0.800 s.
- A2 IMU: 241 samples, 0.000–0.800 s, correct expected shape.
- IMU body gyro versus finite-differenced PhysX attitude: median residual **0.000107775 rad/s**, 95th percentile **0.000190038 rad/s**.
- B0/B1/B3 scans: **8 of 8** files present per condition.
- All 3 saved `pose_cache_source` values: `a2_4a_7_init_7p2_to_8p0_300hz.npz` (full path on local computer).
- In every group, reconstructed Falcon1 scan-body position vs A2 PhysX truth: maximum error **0.0 m**.
- B0 odom: **5** samples at **0.381935–0.781835s**.
- B1 odom: **5** samples at **0.397590–0.797490s**.
- B3 odom: **5** samples at **0.397590–0.797490s**.

Conclusion: The prior concern about A2-B1/B3 selecting the A3 MAPPO cache instead of the A2 geometric-control cache is **resolved by actual NPZ metadata**, despite the generator's default path. This does not by itself verify accelerometer physics/frames or all extrinsics.

## Priority 2 — A3 synchronized five-point window, t=[0.4,0.5,0.6,0.7,0.8] s

| Vehicle | B0 pos RMSE (m) | B1 pos RMSE (m) | B1 improvement | B0 rot RMSE (deg) | B1 rot RMSE (deg) |
|---|---:|---:|---:|---:|---:|
| Falcon1 | 1.035940 | 0.282870 | +72.69% | 1.206 | 2.895 |
| Falcon2 | 0.529156 | 0.888798 | **−67.97%** | 7.951 | 8.472 |
| Falcon3 | 0.515606 | 0.184154 | +64.28% | 2.787 | 2.777 |

These early-window estimates are much worse than the A2-B1 short-window result (~0.01096m), **but A2 and A3 used different trajectories/dynamics and initialization, so no single cause is proven**. The A3 five-point result is re-aligned at 0.4s; the full-run error is aligned at each series' first odometry time (~0.38s), so they represent distinct metrics.

## Priority 3 — A3-B1 drift threshold first crossings

| Vehicle | >0.1m | >0.5m | >1m | >5m |
|---|---:|---:|---:|---:|
| Falcon1 | 0.49759s | 0.99769s | 1.49769s | 3.39789s |
| Falcon2 | 0.49869s | **0.49869s** | 0.69909s | 2.29339s |
| Falcon3 | 0.58579s | 1.18029s | 1.88579s | not observed |

Full-run B1 position RMSE: Falcon1 **5.787539m**, Falcon2 **31.295448m**, Falcon3 **1.421575m**.

Falcon2 exhibits large error almost immediately; this is **not simply late-stage integration drift**. Diagnostic priorities: sensor alignment, timing, gyro/accelerometer consistency with PhysX, coordinate frame/extrinsics, IEKF initialization; later investigate map matching statistics and numerical observability. Do not alter estimator code before confirming inputs.

## Script issue fixed

The first execution `--only a2` created `a2_a3_20261106_offline_audit.json`. The following `--only a3` computed and printed all numbers but raised `FileExistsError` while trying to write the **same** output path. This was a report filename collision, not an experiment failure. The revised script uses `..._a2.json` / `..._a3.json` / `..._all.json` and continues to refuse overwriting existing reports.

## Next steps

1. Run revised `--only a3` to persist A3 JSON; no need to repeat expensive sim/replay.
2. Build a **diagnostic-only** A3 IMU-versus-PhysX check at 300Hz (acceleration, angular velocity and coordinate conventions).
3. Correlate error threshold timestamps against already recorded per-frame IEKF point-match counts and eigenvalues.
4. Only then decide if a controlled single-variable fix is warranted; perform full 5s three-UAV validation later.
