# A4-3a-3 — UAV1 identical-code replay repeatability (2026-11-06)

## Actual observed result

With fixed repaired FAST-LIO2 Odometry covariance publication, same A3-B2 LiDAR/IMU input, `--drone 1 --rate 0.5`, independent replay output saved to:
- `~/research/isaac_a1_diagnostics/A4_3A/fastlio_covariance_replay/drone_1_odom_cov.npz`
- `~/research/isaac_a1_diagnostics/A4_3A/repeat_run_2/drone_1_odom_cov.npz`

User performed full arrays check:

```text
Pose shape: (47, 8) (47, 8)
Covariance shape: (47, 6, 6) (47, 6, 6)
Exact pose equality: True
Exact covariance equality: True
Max pose difference: 0.0
Max covariance difference: 0.0
REPEATABILITY: PASS
```

A preceding timestamp-interpolated comparison yielded 47 matched samples, position RMSE 0 m, rotation RMS 1.49e-15 deg. The direct array equality check is stronger and confirms timestamps, positions, quaternions and all 36 covariance elements exactly agree in saved precision.

## Scientific interpretation

This **passes two-run repeatability for UAV1, on the current configuration**. This does NOT establish accuracy, covariance calibration, cross-UAV covariances, full-system observability, or general deterministic reproducibility. New A4-3a vs OLD A3-B2 trajectory differences of ~0.1423 m/4.90° (UAV1), ~0.1112 m/3.29° (UAV2), and ~0.0658 m/6.06° (UAV3) remain unexplained; current-version UAV1 repeatability is evidence against explaining that difference as ordinary unavoidable replay variation, but cannot prove the exact cause (historic config, initialization, source versions, runtime scheduling).

The validation of 47-frame positive-definite, finite, symmetric, nonzero 6x6 covariances for all 3 drones was already recorded in `A4_3A_THREE_UAV_COVARIANCE_NUMERICAL_ACCEPTANCE_20261106.md`.

## Correct next steps

1. Confirm historical vs new experiment provenance (FAST-LIO2 git revisions, simulated gyro bias zeroing, launch/config and relevant ROS environment), without revising old archives.
2. New-run-based oracle alignment to existing *PhysX truth*, not to old A4-1's LIO trajectory; pose and covariance must come from the same actual new replay.
3. Numerically rotate or otherwise transform internal position covariance into the common evaluation frame with the right Jacobians; account for anchor alignment uncertainty or clearly label conditional oracle-fixed estimates.
4. Evaluate normalized position errors, credible ellipsoids and calibration limitations on 46 temporally correlated frames. Evaluate attitude only after verifying IKFoM SO(3) perturbation convention and frame conversion.
5. Don't describe covariance output as calibrated or ready for full ESKF measurement-R design.
