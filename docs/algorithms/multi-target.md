# Multi-Target Tracking

Multi-target algorithms live under `tracking.multi`. The current MATLAB
implementation focuses on point-target Random Finite Set methods and shared
association/metric utilities.

## RFS Filters

- `tracking.multi.rfs.phd`: legacy IMM/SIMM PHD filters plus migrated
  `GMPHD`, `GMCPHD`, and `PHDFactory`.
- `tracking.multi.rfs.pmbm`: `PMB`, `PMBM`, `PoissonComponent`,
  `MBMComponent`, and `PMBMFactory`.
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
