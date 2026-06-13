# Third-Party Sources

This document records third-party tracking code that is included or explicitly
excluded from Target Tracking Toolbox. Code-level imports are limited to
license-compatible MATLAB sources.

## Integrated

| Source | License | Imported content | Toolbox location | Validation level |
| --- | --- | --- | --- | --- |
| `linh-gist/labeledRFS` | MIT | MATLAB GLMB/LMB Gaussian-mixture examples and helpers from commit `6596faf` | `resources/third_party/matlab/labeledRFS`; adapters under `tracking.multi.rfs.labeled` | Adapter-level numerical smoke |
| `yuhsuansia/Extended-target-PMBM-tracker` | BSD-2-Clause | MATLAB extended-target PMBM source files from commit `9fe871b`, excluding large scenario data files | `resources/third_party/matlab/extended-target-pmbm`; adapters under `tracking.extended.pmbm` | Adapter-level numerical smoke |
| `OmegaEta/Muti-scans-Smoothing-Multiple-Extended-Object-Tracking` | BSD-2-Clause | MATLAB GGIW-PMBM smoother source files from commit `57b0695`, excluding data and figure artifacts | `resources/third_party/matlab/ggiw-pmbm-smoother`; adapter under `tracking.extended.pmbm.smoothing` | Adapter-level numerical smoke |

Original license texts are preserved in `resources/third_party/licenses`.
Vendored MATLAB sources are not added to the public MATLAB path by `startup.m`;
public access goes through the `tracking.*` adapters.
The validation level above means deterministic interface and finite-result
checks on toolbox toy scenarios; it does not imply reproduction of the source
repositories' paper figures.

## Not integrated

| Source | Reason |
| --- | --- |
| `sglvladi/TrackingX` | No clear repository license was found during screening. |
| `shayosler/RFS_tracking` | No clear repository license was found during screening. |
| Vo Codes / `nguyenvanhoa89/tracking/Vo_Codes` | Source states academic/research-purpose usage rather than a standard open-source license. |
| `OmegaEta/NEO-GGIW-PMBM` | GPL-3.0 license is not compatible with direct inclusion in this MIT toolbox. |
