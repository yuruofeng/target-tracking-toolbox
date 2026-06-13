classdef TestMultiRfsFilters < matlab.unittest.TestCase
% TESTMULTIRFSFILTERS  Smoke tests for migrated multi-target RFS filters.

    methods (TestClassSetup)
        function addSourcePath(~)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
        end
    end

    methods (Test)
        function testPointTargetRfsFiltersRunOneStep(testCase)
            config = testCase.createFilterConfig();
            measurements = [1 2; 3 4];

            filterConstructors = {
                @() tracking.multi.rfs.phd.GMPHD(config)
                @() tracking.multi.rfs.phd.GMCPHD(config)
                @() tracking.multi.rfs.pmbm.PMB(config)
                @() tracking.multi.rfs.pmbm.PMBM(config)
            };

            for iFilter = 1:numel(filterConstructors)
                filter = filterConstructors{iFilter}();
                filter = filter.initialize();
                filter = filter.predict();
                filter = filter.update(measurements);
                estimate = filter.estimate();
                testCase.verifyEstimateLike(estimate);
            end
        end

        function testTrajectoryRfsFiltersRunOneStep(testCase)
            config = testCase.createFilterConfig();
            measurements = [1 2; 3 4];

            filterConstructors = {
                @() tracking.multi.rfs.trajectory.phd.GMTPHD(config)
                @() tracking.multi.rfs.trajectory.pmb.TPMB(config)
                @() tracking.multi.rfs.trajectory.pmbm.TPMBM(config)
                @() tracking.multi.rfs.trajectory.mbm.TMBM(config)
            };

            for iFilter = 1:numel(filterConstructors)
                filter = filterConstructors{iFilter}();
                filter = filter.initialize();
                filter = filter.predict();
                filter = filter.update(measurements);
                estimate = filter.estimate();
                testCase.verifyEstimateLike(estimate);
            end
        end

        function testContinuousDiscreteFiltersUseClutterIntensity(testCase)
            config = testCase.createFilterConfig();
            config.clutterIntensity = 1e-4;
            measurements = [1 2; 3 4];

            filterConstructors = {
                @() tracking.multi.rfs.cd.CDGMPHD(config)
                @() tracking.multi.rfs.cd.CDGMCPHD(config)
                @() tracking.multi.rfs.cd.CDPMBM(config)
            };

            for iFilter = 1:numel(filterConstructors)
                filter = filterConstructors{iFilter}();
                filter = filter.initialize();
                filter = filter.predict(1);
                filter = filter.update(measurements);
                estimate = filter.estimate();
                testCase.verifyTrue(isnumeric(estimate) || isstruct(estimate));
            end
        end
    end

    methods
        function config = createFilterConfig(~)
            config = tracking.multi.rfs.core.FilterConfig();
            config.detectionProb = 0.9;
            config.survivalProb = 0.99;
            config.clutterRate = 1e-4;
            config.surveillanceArea = [1000, 1000];
            config.pruningThreshold = 1e-5;
            config.maxComponents = 50;
            config.gatingThreshold = 20;
            config.existenceThreshold = 1e-5;

            config.motionModel = struct( ...
                'type', 'CV', ...
                'F', eye(4), ...
                'Q', eye(4));
            config.measurementModel = struct( ...
                'type', 'Linear', ...
                'H', [1 0 0 0; 0 1 0 0], ...
                'R', eye(2));
            config.birthModel = struct( ...
                'type', 'Poisson', ...
                'intensity', 0.005, ...
                'means', zeros(4, 1), ...
                'covs', eye(4), ...
                'weights', 1);

            config.extraParams.totalTimeSteps = 10;
            config.extraParams.gateSize = 9.210;
            config.extraParams.maxGlobalHypotheses = 100;
            config.extraParams.minGlobalHypothesisWeight = 1e-4;
            config.extraParams.numMinimumAssignment = 10;
            config.extraParams.minEndTimeProbability = 1e-4;
            config.extraParams.minBirthTimeProbability = 1e-1;
            config.extraParams.minExistenceProbability = 1e-4;
        end

        function verifyEstimateLike(testCase, estimate)
            testCase.verifyTrue(isstruct(estimate));
            testCase.verifyTrue(isfield(estimate, 'states'));
            testCase.verifyTrue(isfield(estimate, 'cardinality'));
        end
    end
end
