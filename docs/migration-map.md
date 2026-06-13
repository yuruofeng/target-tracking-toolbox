# Migration Map

| Legacy path or symbol | New path or symbol | Notes |
| --- | --- | --- |
| `+dbt/*` | `src/matlab/+tracking/+single/+dbt/*` | Single-target DBT filters and maneuver models |
| `dbt.EKF` | `tracking.single.dbt.EKF` | Same implementation, new namespace |
| `dbt.IMM` | `tracking.single.dbt.IMM` | Same implementation, new namespace |
| `+tbd/*` | `src/matlab/+tracking/+single/+tbd/*` | Single-target TBD algorithms |
| `tbd.DpTbd` | `tracking.single.tbd.DpTbd` | Same implementation, new namespace |
| `+phd/*` | `src/matlab/+tracking/+multi/+rfs/+phd/*` | Reclassified as multi-target RFS/PHD |
| `phd.ImmPhdFilter` | `tracking.multi.rfs.phd.ImmPhdFilter` | Same implementation, new namespace |
| `05-extended-target-tracking/+ggiw/*` | `src/matlab/+tracking/+extended/+ggiw/*` | Extended-target GGIW module |
| `05-extended-target-tracking/+starconvex/*` | `src/matlab/+tracking/+extended/+starconvex/*` | Extended-target shape tracking |
| `05-extended-target-tracking/+phd/*` | `src/matlab/+tracking/+extended/+phd/*` | Extended-target PHD module |
| `+utils/FilterUtils.m` | `tracking.core.FilterUtils` | Numeric filtering utilities |
| `+utils/MeasurementModel.m` | `tracking.models.MeasurementModel` | Motion and measurement helpers |
| `+utils/OspaMetric.m` | `tracking.metrics.OspaMetric` | Evaluation metric |
| `+utils/Hungarian.m` | `tracking.association.Hungarian` | Assignment utility |
| `+viz/Visualizer.m` | `tracking.viz.Visualizer` | Visualization helper |
| `demos/*.m` | `examples/matlab/*.m` | Demonstrations now use semantic names |
| `tests/*.m` | `tests/matlab/unit/*.m` | MATLAB unit tests |
| `05-extended-target-tracking/tests/test_all.m` | `tests/matlab/integration/test_extended_target_all.m` | Extended-target integration test |

## multiple-target-tracking-main

| Legacy path or symbol | New path or symbol | Notes |
| --- | --- | --- |
| `multiple-target-tracking-main/+assignment/AssignmentFactory.m` | `src/matlab/+tracking/+association/AssignmentFactory.m` | Factory for assignment solvers; covered by `TestMultiAssociationMetrics` |
| `multiple-target-tracking-main/+assignment/Auction.m` | `src/matlab/+tracking/+association/Auction.m` | Assignment solver; call `tracking.association.Auction` |
| `multiple-target-tracking-main/+assignment/Munkres.m` | `src/matlab/+tracking/+association/Munkres.m` | Assignment solver; call `tracking.association.Munkres` |
| `multiple-target-tracking-main/+assignment/Murty.m` | `src/matlab/+tracking/+association/Murty.m` | K-best assignment solver; call `tracking.association.Murty` |
| `multiple-target-tracking-main/+metric/GOSPA.m` | `src/matlab/+tracking/+metrics/GOSPA.m` | Multi-target metric; call `tracking.metrics.GOSPA` |
| `multiple-target-tracking-main/+metric/MetricFactory.m` | `src/matlab/+tracking/+metrics/MetricFactory.m` | Metric factory; call `tracking.metrics.MetricFactory` |
| `multiple-target-tracking-main/+metric/TrajectoryErrorCalculator.m` | `src/matlab/+tracking/+metrics/TrajectoryErrorCalculator.m` | Trajectory error helper; call `tracking.metrics.TrajectoryErrorCalculator` |
| `multiple-target-tracking-main/+metric/TrajectoryMetric.m` | `src/matlab/+tracking/+metrics/TrajectoryMetric.m` | Trajectory metric; call `tracking.metrics.TrajectoryMetric` |
| `multiple-target-tracking-main/+utils/ErrorCode.m` | `src/matlab/+tracking/+core/ErrorCode.m` | Core error enumeration; call `tracking.core.ErrorCode` |
| `multiple-target-tracking-main/+utils/MTTException.m` | `src/matlab/+tracking/+core/MTTException.m` | Core exception type; call `tracking.core.MTTException` |
| `multiple-target-tracking-main/+utils/BaseFilter.m` | `src/matlab/+tracking/+multi/+rfs/+core/BaseFilter.m` | RFS filter base class; call `tracking.multi.rfs.core.BaseFilter` |
| `multiple-target-tracking-main/+utils/FilterConfig.m` | `src/matlab/+tracking/+multi/+rfs/+core/FilterConfig.m` | RFS filter configuration; call `tracking.multi.rfs.core.FilterConfig` |
| `multiple-target-tracking-main/+utils/FilterResult.m` | `src/matlab/+tracking/+multi/+rfs/+core/FilterResult.m` | RFS internal result object; call `tracking.multi.rfs.core.FilterResult` |
| `multiple-target-tracking-main/+phd/GMPHD.m` | `src/matlab/+tracking/+multi/+rfs/+phd/GMPHD.m` | Gaussian-mixture PHD; call `tracking.multi.rfs.phd.GMPHD` |
| `multiple-target-tracking-main/+phd/GMCPHD.m` | `src/matlab/+tracking/+multi/+rfs/+phd/GMCPHD.m` | Gaussian-mixture CPHD; call `tracking.multi.rfs.phd.GMCPHD` |
| `multiple-target-tracking-main/+phd/PHDFactory.m` | `src/matlab/+tracking/+multi/+rfs/+phd/PHDFactory.m` | PHD-family factory; call `tracking.multi.rfs.phd.PHDFactory` |
| `multiple-target-tracking-main/+pmbm/MBMComponent.m` | `src/matlab/+tracking/+multi/+rfs/+pmbm/MBMComponent.m` | PMBM support component; call `tracking.multi.rfs.pmbm.MBMComponent` |
| `multiple-target-tracking-main/+pmbm/PMB.m` | `src/matlab/+tracking/+multi/+rfs/+pmbm/PMB.m` | PMB filter; call `tracking.multi.rfs.pmbm.PMB` |
| `multiple-target-tracking-main/+pmbm/PMBM.m` | `src/matlab/+tracking/+multi/+rfs/+pmbm/PMBM.m` | PMBM filter; call `tracking.multi.rfs.pmbm.PMBM` |
| `multiple-target-tracking-main/+pmbm/PMBMFactory.m` | `src/matlab/+tracking/+multi/+rfs/+pmbm/PMBMFactory.m` | PMBM-family factory; call `tracking.multi.rfs.pmbm.PMBMFactory` |
| `multiple-target-tracking-main/+pmbm/PoissonComponent.m` | `src/matlab/+tracking/+multi/+rfs/+pmbm/PoissonComponent.m` | PMBM support component; call `tracking.multi.rfs.pmbm.PoissonComponent` |
| `multiple-target-tracking-main/+cdfilters/CDFilterFactory.m` | `src/matlab/+tracking/+multi/+rfs/+cd/CDFilterFactory.m` | Continuous-discrete factory; call `tracking.multi.rfs.cd.CDFilterFactory` |
| `multiple-target-tracking-main/+cdfilters/CDGMCPHD.m` | `src/matlab/+tracking/+multi/+rfs/+cd/CDGMCPHD.m` | Continuous-discrete GM-CPHD; call `tracking.multi.rfs.cd.CDGMCPHD` |
| `multiple-target-tracking-main/+cdfilters/CDGMPHD.m` | `src/matlab/+tracking/+multi/+rfs/+cd/CDGMPHD.m` | Continuous-discrete GM-PHD; call `tracking.multi.rfs.cd.CDGMPHD` |
| `multiple-target-tracking-main/+cdfilters/CDPMBM.m` | `src/matlab/+tracking/+multi/+rfs/+cd/CDPMBM.m` | Continuous-discrete PMBM; call `tracking.multi.rfs.cd.CDPMBM` |
| `multiple-target-tracking-main/+tphd/GMTPHD.m` | `src/matlab/+tracking/+multi/+rfs/+trajectory/+phd/GMTPHD.m` | Trajectory PHD; call `tracking.multi.rfs.trajectory.phd.GMTPHD` |
| `multiple-target-tracking-main/+tphd/TPHDFactory.m` | `src/matlab/+tracking/+multi/+rfs/+trajectory/+phd/TPHDFactory.m` | Trajectory PHD factory; call `tracking.multi.rfs.trajectory.phd.TPHDFactory` |
| `multiple-target-tracking-main/+tpmb/TPMB.m` | `src/matlab/+tracking/+multi/+rfs/+trajectory/+pmb/TPMB.m` | Trajectory PMB; call `tracking.multi.rfs.trajectory.pmb.TPMB` |
| `multiple-target-tracking-main/+tpmbm/TPMBFactory.m` | `src/matlab/+tracking/+multi/+rfs/+trajectory/+pmbm/TPMBFactory.m` | Trajectory PMB factory; call `tracking.multi.rfs.trajectory.pmbm.TPMBFactory` |
| `multiple-target-tracking-main/+tpmbm/TPMBM.m` | `src/matlab/+tracking/+multi/+rfs/+trajectory/+pmbm/TPMBM.m` | Trajectory PMBM; call `tracking.multi.rfs.trajectory.pmbm.TPMBM` |
| `multiple-target-tracking-main/+tpmbm/TPMBMFactory.m` | `src/matlab/+tracking/+multi/+rfs/+trajectory/+pmbm/TPMBMFactory.m` | Trajectory PMBM factory; call `tracking.multi.rfs.trajectory.pmbm.TPMBMFactory` |
| `multiple-target-tracking-main/+tmbm/TMBM.m` | `src/matlab/+tracking/+multi/+rfs/+trajectory/+mbm/TMBM.m` | Trajectory MBM; call `tracking.multi.rfs.trajectory.mbm.TMBM` |
| `multiple-target-tracking-main/+viz/TrackingVisualizer.m` | `src/matlab/+tracking/+viz/TrackingVisualizer.m` | Multi-target visualizer; call `tracking.viz.TrackingVisualizer` |
| `multiple-target-tracking-main/+viz/TrajectoryVisualizer.m` | `src/matlab/+tracking/+viz/TrajectoryVisualizer.m` | Trajectory visualizer; call `tracking.viz.TrajectoryVisualizer` |
| `multiple-target-tracking-main/demos/demo_refactored_filters.m` | `examples/matlab/demo_multi_rfs_filters.m` | Example now emits `TrackingResult`-compatible structs into `results/` |
| `multiple-target-tracking-main/demos/demo_filter_comparison.m` | `examples/matlab/demo_multi_filter_comparison.m` | Example now emits comparison results into `results/` |
| `multiple-target-tracking-main/tests/*.m` | `tests/matlab/unit/TestMulti*.m` | Smoke and integration checks converted to `matlab.unittest` assertions |

No compatibility shims are kept for the legacy top-level package names.
