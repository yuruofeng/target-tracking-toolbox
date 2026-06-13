# Architecture

The toolbox is organized around task families rather than historical folders.
MATLAB remains the primary runtime for classic filters, simulation, evaluation
and visualization. Python is an optional backend for future learning-based
trackers.

## Package Layers

- `tracking.single`: point-target single-object tracking. DBT and TBD methods
  live here.
- `tracking.multi`: multi-object tracking. RFS point-target filters live under
  `tracking.multi.rfs`, including PHD/CPHD, PMB/PMBM, trajectory RFS, and
  continuous-discrete variants.
- `tracking.extended`: extended-target tracking, including GGIW, star-convex
  shape tracking and extended-target PHD code.
- `tracking.core`: small shared helpers and toolbox metadata.
- `tracking.multi.rfs.core`: multi-target RFS-specific configuration, base
  filter, and transition result classes.
- `tracking.models`, `tracking.metrics`, `tracking.association`, `tracking.viz`:
  reusable support modules shared by the task families.
- `tracking.learning`: MATLAB adapters for Python models.

## Extension Rules

New methods should expose a small runnable surface:

- `initialize(obj, initialCondition)` when the method has persistent state.
- `predict(obj, stepContext)` for time updates.
- `update(obj, measurements, stepContext)` for measurement updates.
- `estimate(obj)` for current estimates.
- `run(obj, scenarioOrData, varargin)` for batch demonstrations and tests.

Existing algorithms do not need to inherit one monolithic base class. Prefer thin
adapters around legacy implementations until repeated behavior justifies a new
shared abstraction.

## Result Contract

Use `tracking.core.createTrackingResult` for new MATLAB demos and return the
same keys from Python:

`metadata`, `truth`, `measurements`, `estimates`, `metrics`, `config`.
