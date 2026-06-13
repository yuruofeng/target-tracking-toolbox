# Single-Target Tracking

Single-target algorithms live under `tracking.single`.

- `tracking.single.dbt`: detect-before-track filters including EKF, UKF, CKF,
  particle filter, motion-model EKF and IMM.
- `tracking.single.tbd`: track-before-detect algorithms including DP-TBD and
  PF-TBD.

Examples:

```matlab
run('examples/matlab/demo_single_dbt.m');
run('examples/matlab/demo_single_tbd.m');
run('examples/matlab/demo_single_maneuver.m');
```
