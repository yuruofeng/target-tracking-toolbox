classdef ProjectConfig
% TRACKING.CORE.PROJECTCONFIG  Toolbox metadata and package registry.

    properties (Constant)
        PROJECT_NAME = 'Target Tracking Toolbox'
        PROJECT_VERSION = '2.1.0'
        PROJECT_AUTHOR = 'Target Tracking Team'

        PACKAGES = struct( ...
            'SingleDbt', 'tracking.single.dbt', ...
            'SingleTbd', 'tracking.single.tbd', ...
            'MultiPhd', 'tracking.multi.rfs.phd', ...
            'ExtendedGgiw', 'tracking.extended.ggiw', ...
            'ExtendedStarConvex', 'tracking.extended.starconvex', ...
            'Metrics', 'tracking.metrics', ...
            'Association', 'tracking.association', ...
            'Models', 'tracking.models', ...
            'Visualization', 'tracking.viz')

        SINGLE_DBT_FILTERS = {'EKF', 'UKF', 'CKF', 'ParticleFilter', 'IMM', ...
                              'MotionModelEKF'}
        SINGLE_TBD_ALGORITHMS = {'DpTbd', 'PfTbd'}
        MULTI_RFS_PHD_FILTERS = {'ImmPhdFilter', 'SimmPhdFilter'}
        EXTENDED_TARGET_FILTERS = {'GgiwFilter', 'StarConvexTracker', ...
                                   'ExtendedTargetPhdFilter'}
    end

    methods (Static)
        function displayInfo()
            fprintf('\n');
            fprintf('====================================================\n');
            fprintf('  %s v%s\n', tracking.core.ProjectConfig.PROJECT_NAME, ...
                tracking.core.ProjectConfig.PROJECT_VERSION);
            fprintf('  Author: %s\n', tracking.core.ProjectConfig.PROJECT_AUTHOR);
            fprintf('====================================================\n\n');

            fprintf('Packages:\n');
            fprintf('  tracking.single.dbt      Single-target DBT filters\n');
            fprintf('  tracking.single.tbd      Single-target TBD algorithms\n');
            fprintf('  tracking.multi.rfs.phd   Multi-target RFS/PHD filters\n');
            fprintf('  tracking.extended        Extended-target algorithms\n');
            fprintf('  tracking.metrics         Evaluation metrics\n');
            fprintf('  tracking.association     Assignment and association utilities\n');
            fprintf('  tracking.models          Motion and measurement models\n');
            fprintf('  tracking.viz             Visualization utilities\n\n');

            fprintf('Quick start:\n');
            fprintf('  run(''examples/matlab/demo_single_dbt.m'')\n');
            fprintf('  run(''examples/matlab/demo_single_tbd.m'')\n');
            fprintf('  run(''examples/matlab/demo_multi_phd.m'')\n');
            fprintf('  run(''tests/matlab/runAllTests.m'')\n');
            fprintf('====================================================\n\n');
        end

        function listAlgorithms()
            fprintf('\nSingle-target DBT:\n');
            tracking.core.ProjectConfig.printNames( ...
                tracking.core.ProjectConfig.PACKAGES.SingleDbt, ...
                tracking.core.ProjectConfig.SINGLE_DBT_FILTERS);

            fprintf('\nSingle-target TBD:\n');
            tracking.core.ProjectConfig.printNames( ...
                tracking.core.ProjectConfig.PACKAGES.SingleTbd, ...
                tracking.core.ProjectConfig.SINGLE_TBD_ALGORITHMS);

            fprintf('\nMulti-target RFS/PHD:\n');
            tracking.core.ProjectConfig.printNames( ...
                tracking.core.ProjectConfig.PACKAGES.MultiPhd, ...
                tracking.core.ProjectConfig.MULTI_RFS_PHD_FILTERS);

            fprintf('\nExtended target:\n');
            fprintf('    - tracking.extended.ggiw.GgiwFilter\n');
            fprintf('    - tracking.extended.starconvex.StarConvexTracker\n');
            fprintf('    - tracking.extended.phd.ExtendedTargetPhdFilter\n');
        end

        function root = getProjectRoot()
            currentFile = mfilename('fullpath');
            root = fileparts(fileparts(fileparts(fileparts(fileparts(currentFile)))));
        end

        function setupPath()
            root = tracking.core.ProjectConfig.getProjectRoot();
            addpath(genpath(fullfile(root, 'src', 'matlab')));
            fprintf('MATLAB source path configured: %s\n', ...
                fullfile(root, 'src', 'matlab'));
        end
    end

    methods (Static, Access = private)
        function printNames(packageName, names)
            for iName = 1:numel(names)
                fprintf('    - %s.%s\n', packageName, names{iName});
            end
        end
    end
end
