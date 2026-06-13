# Multi-Target Tracking

Multi-target algorithms live under `tracking.multi`. The current MATLAB
implementation focuses on point-target Random Finite Set methods and shared
association/metric utilities.

## RFS Filters

- `tracking.multi.rfs.phd`: legacy IMM/SIMM PHD filters plus migrated
  `GMPHD`, `GMCPHD`, and `PHDFactory`.
- `tracking.multi.rfs.pmbm`: `PMB`, `PMBM`, `PoissonComponent`,
  `MBMComponent`, and `PMBMFactory`.
- `tracking.multi.rfs.labeled.glmb`: `JointGlmbGms`, a MATLAB adapter for
  the MIT-licensed labeledRFS joint GLMB Gaussian-mixture source.
- `tracking.multi.rfs.labeled.lmb`: `JointLmbGms`, a MATLAB adapter for
  the MIT-licensed labeledRFS joint LMB Gaussian-mixture source.
- `tracking.multi.rfs.trajectory.phd`: trajectory PHD (`GMTPHD`).
- `tracking.multi.rfs.trajectory.pmb`: trajectory PMB (`TPMB`).
- `tracking.multi.rfs.trajectory.pmbm`: trajectory PMBM factories and filter.
- `tracking.multi.rfs.trajectory.mbm`: trajectory MBM (`TMBM`).
- `tracking.multi.rfs.cd`: continuous-discrete `CDGMPHD`, `CDGMCPHD`, and
  `CDPMBM` filters.

The migrated filters keep their lightweight MATLAB object workflow:
`initialize`, `predict`, `update`, `estimate`, and optional `run`.

## Support Modules

- `tracking.association`: Hungarian, Murty, Auction, Munkres, and factory
  helpers.
- `tracking.metrics`: OSPA, GOSPA, trajectory metrics, and trajectory error
  calculations.
- `tracking.viz`: target and trajectory visualization helpers.

New examples should return `tracking.core.createTrackingResult` structs and
write outputs under `results/`.

## Validation Level

Multi-target point and trajectory RFS filters are covered by deterministic
smoke plus numerical regression on fixed toy scenarios. The checks verify that
`initialize -> predict -> update -> estimate` completes, estimates are finite or
legitimate empty target sets, cardinality remains bounded, and OSPA/GOSPA stay
within the configured cutoff-scale acceptance range. Labeled RFS adapters are
validated at adapter-level numerical smoke with stable `TrackingResult`
structure, labels, scores, covariances and finite GOSPA.
