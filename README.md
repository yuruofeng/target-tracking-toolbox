# Target Tracking Toolbox

The repository organizes single-target DBT/TBD,
multi-target RFS tracking, extended-target tracking, shared metrics,
association utilities, visualization helpers, and learning adapters under one
public namespace: `tracking.*`.

The newly integrated multi-target RFS code is based on the
`multiple-target-tracking-main` implementation derived from public examples by
A. Garcia-Fernandez and related multi-target tracking literature.

Validation currently means deterministic smoke tests plus small-scenario
numerical regression for public algorithms; it does not claim paper-level curve
reproduction.

## Quick Start

```matlab
cd('target-tracking-toolbox');
startup();

run('examples/matlab/demo_single_dbt.m');
run('examples/matlab/demo_single_tbd.m');
run('examples/matlab/demo_multi_phd.m');
run('examples/matlab/demo_multi_rfs_filters.m');
run('examples/matlab/demo_multi_filter_comparison.m');
run('examples/matlab/demo_multi_labeled_rfs.m');
run('examples/matlab/demo_extended_ggiw.m');
run('examples/matlab/demo_extended_starconvex.m');
run('examples/matlab/demo_extended_phd.m');
run('examples/matlab/demo_extended_pmbm.m');
run('examples/matlab/demo_extended_pmbm_smoothing.m');
run('tests/matlab/runAllTests.m');
```

Python extension smoke test:

```powershell
$env:PYTHONPATH = "python/src"
python -m pytest tests/python
```

## Repository Layout

```text
src/matlab/+tracking/
|-- +single/+dbt                 # EKF, UKF, CKF, PF, IMM, maneuver models
|-- +single/+tbd                 # DP-TBD and PF-TBD
|-- +multi/+rfs/+phd             # IMM/SIMM PHD plus GM-PHD/GM-CPHD
|-- +multi/+rfs/+pmbm            # PMB and PMBM filters
|-- +multi/+rfs/+labeled         # GLMB/LMB adapters
|-- +multi/+rfs/+trajectory      # TPHD, TPMB, TPMBM, TMBM filters
|-- +multi/+rfs/+cd              # continuous-discrete GM-PHD/GM-CPHD/PMBM
|-- +multi/+rfs/+core            # multi-target config/result/base classes
|-- +extended                    # GGIW, star-convex and extended-target PHD
|-- +core                        # toolbox metadata, exceptions and result helpers
|-- +models                      # measurement and motion model utilities
|-- +metrics                     # OSPA, GOSPA and trajectory metrics
|-- +association                 # Hungarian, Murty, Auction and Munkres
|-- +viz                         # plotting utilities
|-- +io                          # JSON configuration loading
`-- +learning                    # MATLAB adapter for Python learning backends
```

Other important directories:

- `configs/`: JSON configuration examples shared by MATLAB and Python.
- `examples/matlab/`: runnable demonstrations with output written to `results/`.
- `python/`: future deep-learning package, currently a minimal adapter contract.
- `tests/matlab/`: MATLAB unit, integration and regression tests.
- `docs/`: architecture, migration map and algorithm notes.

## Public MATLAB Namespace

| Area                    | Namespace                         | Examples                                           |
| ----------------------- | --------------------------------- | -------------------------------------------------- |
| Single-target DBT       | `tracking.single.dbt`             | `EKF`, `UKF`, `CKF`, `ParticleFilter`, `IMM`       |
| Single-target TBD       | `tracking.single.tbd`             | `DpTbd`, `PfTbd`                                   |
| Multi-target PHD/CPHD   | `tracking.multi.rfs.phd`          | `ImmPhdFilter`, `SimmPhdFilter`, `GMPHD`, `GMCPHD` |
| Multi-target PMB/PMBM   | `tracking.multi.rfs.pmbm`         | `PMB`, `PMBM`, `PMBMFactory`                       |
| Labeled RFS             | `tracking.multi.rfs.labeled.*`    | `JointGlmbGms`, `JointLmbGms`                      |
| Trajectory RFS          | `tracking.multi.rfs.trajectory.*` | `GMTPHD`, `TPMB`, `TPMBM`, `TMBM`                  |
| Continuous-discrete RFS | `tracking.multi.rfs.cd`           | `CDGMPHD`, `CDGMCPHD`, `CDPMBM`                    |
| Extended target         | `tracking.extended.*`             | `GgiwFilter`, `StarConvexTracker`, `ExtendedTargetPhdFilter`, `TargetPmbmFilter` |
| Metrics                 | `tracking.metrics`                | `OspaMetric`, `GOSPA`, `TrajectoryMetric`          |
| Association             | `tracking.association`            | `Hungarian`, `Murty`, `Auction`, `Munkres`         |
| Models                  | `tracking.models`                 | `MeasurementModel`                                 |
| Learning                | `tracking.learning`               | `PythonModelAdapter`                               |

## Shared Result Contract

New demos and adapters should return a TrackingResult-compatible struct or dict:

- `metadata`: algorithm name, task type, version, seed and backend.
- `truth`: ground truth trajectory or target sets.
- `measurements`: measurement sequence.
- `estimates`: estimated states, covariances, labels, scores and extents.
- `metrics`: RMSE, OSPA, GOSPA, cardinality error and runtime.
- `config`: resolved configuration snapshot.

MATLAB helper:

```matlab
result = tracking.core.createTrackingResult(metadata, truth, measurements, ...
    estimates, metrics, config);
```

## Configuration

JSON is the primary cross-language configuration format:

```matlab
cfg = tracking.io.loadConfig('configs/multi/pmbm_default.json');
```

MATLAB-only scripts may still construct rich configuration objects directly, but
new reusable examples should prefer JSON when the same scenario may later be
used by Python learning code.

## Development

Run MATLAB tests:

```matlab
startup();
results = runAllTests();
assertSuccess(results);
```

Run only numerical regression tests:

```matlab
startup();
addpath('tests/matlab/regression');
results = runRegressionTests();
assertSuccess(results);
```

Run Python tests:

```powershell
$env:PYTHONPATH = "python/src"
python -m pytest tests/python
```

See `docs/architecture.md`, `docs/migration-map.md`, and
`docs/third-party-sources.md` before adding new multi-target or deep-learning
methods.
