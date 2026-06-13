function startup()
% STARTUP  Initialize the Target Tracking Toolbox MATLAB environment.
%
%   Usage:
%       cd('target-tracking-toolbox');
%       startup();
%
%   Public packages after initialization:
%       tracking.single.dbt      - single-target detect-before-track filters
%       tracking.single.tbd      - single-target track-before-detect methods
%       tracking.multi.rfs.phd   - multi-target PHD/CPHD filters
%       tracking.multi.rfs.pmbm  - multi-target PMB/PMBM filters
%       tracking.multi.rfs.cd    - continuous-discrete RFS filters
%       tracking.multi.rfs.trajectory - trajectory RFS filters
%       tracking.extended        - extended-target tracking algorithms
%       tracking.metrics         - tracking metrics such as OSPA
%       tracking.association     - data-association utilities
%       tracking.models          - motion and measurement models
%       tracking.viz             - visualization helpers

    repoRoot = fileparts(mfilename('fullpath'));
    if isempty(repoRoot)
        repoRoot = pwd;
    end

    matlabRoot = fullfile(repoRoot, 'src', 'matlab');
    addpath(genpath(matlabRoot));

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('  Target Tracking Toolbox v2.1.0\n');
    fprintf('============================================================\n');
    fprintf('  MATLAB source: %s\n', matlabRoot);
    fprintf('\n');
    fprintf('  Packages:\n');
    fprintf('    tracking.single.dbt      DBT filters: EKF, UKF, CKF, IMM, PF\n');
    fprintf('    tracking.single.tbd      TBD methods: DP-TBD, PF-TBD\n');
    fprintf('    tracking.multi.rfs.phd   Multi-target PHD and CPHD filters\n');
    fprintf('    tracking.multi.rfs.pmbm  Multi-target PMB and PMBM filters\n');
    fprintf('    tracking.multi.rfs.cd    Continuous-discrete RFS filters\n');
    fprintf('    tracking.multi.rfs.trajectory  Trajectory PHD/PMB/PMBM/MBM filters\n');
    fprintf('    tracking.extended        Extended-target GGIW, star-convex, PHD\n');
    fprintf('    tracking.metrics         OSPA and related metrics\n');
    fprintf('    tracking.association     Assignment and association utilities\n');
    fprintf('\n');
    fprintf('  Quick start:\n');
    fprintf('    run(''examples/matlab/demo_single_dbt.m'')\n');
    fprintf('    run(''examples/matlab/demo_single_tbd.m'')\n');
    fprintf('    run(''examples/matlab/demo_multi_phd.m'')\n');
    fprintf('    run(''examples/matlab/demo_multi_rfs_filters.m'')\n');
    fprintf('    run(''examples/matlab/demo_multi_filter_comparison.m'')\n');
    fprintf('    run(''tests/matlab/runAllTests.m'')\n');
    fprintf('============================================================\n\n');
end
