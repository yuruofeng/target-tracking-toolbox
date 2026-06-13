# Extended-Target Tracking

Extended-target algorithms live under `tracking.extended`.

- `tracking.extended.ggiw`: GGIW-PHD adapter and extended measurement utilities.
- `tracking.extended.starconvex`: star-convex shape tracker adapter and generators.
- `tracking.extended.phd`: extended-target PHD adapter and measurement partition.
- `tracking.extended.pmbm`: extended-target PMBM target and trajectory adapters.
- `tracking.extended.pmbm.smoothing`: GGIW-PMBM smoothing adapter.
- `tracking.extended.utils`: utilities local to the extended-target module.

The public GGIW, star-convex and extended-PHD classes expose the same adapter
surface as the rest of the toolbox: `initialize`, `predict`, `update`,
`estimate`, and `run`. Their original MATLAB implementations live under
`tracking.extended.internal` and are not documented as public APIs.

Examples:

```matlab
run('examples/matlab/demo_extended_ggiw.m');
run('examples/matlab/demo_extended_starconvex.m');
run('examples/matlab/demo_extended_phd.m');
run('examples/matlab/demo_extended_pmbm.m');
run('examples/matlab/demo_extended_pmbm_smoothing.m');
```

## Validation Level

Extended-target algorithms are covered by deterministic smoke plus numerical
regression on small scenarios. The checks verify finite states and covariances,
labels, square positive-semidefinite extents, and `TrackingResult` compatibility
for PMBM and smoother adapters. Third-party extended PMBM and GGIW-PMBM smoother
code is currently validated at adapter-level numerical smoke, not paper-level
reproduction of the original experiments.
