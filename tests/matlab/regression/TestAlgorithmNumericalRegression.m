classdef TestAlgorithmNumericalRegression < matlab.unittest.TestCase
% TESTALGORITHMNUMERICALREGRESSION  Deterministic numerical regression tests.

    properties (Constant)
        RegressionSeed = 20260612
        RuntimeLimitSeconds = 180
    end

    methods (TestClassSetup)
        function addSourcePath(~)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
            addpath(fullfile(repoRoot, 'tests', 'matlab', 'regression'));
        end
    end

    methods (Test)
        function testSingleTargetDbtRegression(testCase)
            rng(testCase.RegressionSeed);
            config = tracking.single.dbt.Config('numSteps', 50, 'numParticles', 500);
            scenario = tracking.single.dbt.Scenario(config);
            [truthX, meas] = scenario.generate();
            x0 = config.initState;
            P0 = blkdiag(10 * eye(4), pi / 90);

            algorithms = {
                'EKF', @() tracking.single.dbt.EKF(config), 50
                'UKF', @() tracking.single.dbt.UKF(config), 50
                'CKF', @() tracking.single.dbt.CKF(config), 50
                'ParticleFilter', @() tracking.single.dbt.ParticleFilter(config), 150
            };

            for iAlgorithm = 1:size(algorithms, 1)
                filter = algorithms{iAlgorithm, 2}();
                [estStates, estCovars, elapsed] = filter.run(meas, x0, P0);

                testCase.verifySize(estStates, [config.stateDim, config.numSteps], algorithms{iAlgorithm, 1});
                testCase.verifySize(estCovars, [config.stateDim, config.stateDim, config.numSteps]);
                localVerifyFinite(testCase, estStates, algorithms{iAlgorithm, 1});
                testCase.verifyGreaterThanOrEqual(elapsed, 0);

                posRmse = localRmse(estStates([1, 3], :), truthX([1, 3], :), 1:2);
                testCase.verifyLessThan(posRmse, algorithms{iAlgorithm, 3}, algorithms{iAlgorithm, 1});
            end
        end

        function testTbdRegression(testCase)
            rng(testCase.RegressionSeed);
            config = tracking.single.tbd.Config( ...
                'numFrames', 20, ...
                'numParticles', 100, ...
                'gridSize', [50, 50]);
            rng(config.rngSeed);
            scenario = tracking.single.tbd.Scenario(config);
            [~, truth, measData, psfKernel] = scenario.generate();

            dp = tracking.single.tbd.DpTbd(config);
            dp.run(measData, psfKernel);
            [dpTrack, dpScore] = dp.getResults();
            testCase.verifySize(dpTrack, [config.numFrames, 2]);
            localVerifyFinite(testCase, dpTrack, 'DpTbd');
            testCase.verifyGreaterThan(dpScore, -inf);
            testCase.verifyLessThan(localRmse(dpTrack', truth(:, 1:2)', 1:2), 10);

            pf = tracking.single.tbd.PfTbd(config);
            pf.run(measData, truth(1, :), psfKernel);
            pfState = pf.getEstimate();
            [pfPosRmse, ~] = pf.computeRmse(truth);
            testCase.verifySize(pfState, [config.numFrames, 5]);
            localVerifyFinite(testCase, pfState, 'PfTbd');
            testCase.verifyLessThan(mean(pfPosRmse), 10);
        end

        function testManeuverRegression(testCase)
            rng(testCase.RegressionSeed);
            numSteps = 50;
            configManeuver = tracking.single.dbt.ConfigManeuver('numSteps', numSteps, 'dt', 1);
            scenario = tracking.single.dbt.ScenarioManeuver(configManeuver);
            [truthX, meas] = scenario.generate();

            modelSpecs = {
                'CV', 4, @() [truthX(1, 1); truthX(2, 1); truthX(4, 1); truthX(5, 1)], diag([100, 25, 100, 25])
                'CA', 6, @() [truthX(1, 1); truthX(2, 1); 0; truthX(4, 1); truthX(5, 1); 0], diag([100, 25, 25, 100, 25, 25])
                'CT', 5, @() [truthX(1, 1); truthX(2, 1); truthX(4, 1); truthX(5, 1); 0.01], diag([100, 25, 100, 25, 0.01])
                'Singer', 6, @() [truthX(1, 1); truthX(2, 1); 0; truthX(4, 1); truthX(5, 1); 0], diag([100, 25, 25, 100, 25, 25])
                'CS', 6, @() [truthX(1, 1); truthX(2, 1); 0; truthX(4, 1); truthX(5, 1); 0], diag([100, 25, 25, 100, 25, 25])
            };

            for iModel = 1:size(modelSpecs, 1)
                cfg = tracking.single.dbt.MotionModelConfig(modelSpecs{iModel, 1}, 'numSteps', numSteps, 'dt', 1);
                filter = tracking.single.dbt.MotionModelEKF(cfg);
                [estStates, ~, elapsed] = filter.run(meas, modelSpecs{iModel, 3}(), modelSpecs{iModel, 4});
                testCase.verifySize(estStates, [modelSpecs{iModel, 2}, numSteps], modelSpecs{iModel, 1});
                localVerifyFinite(testCase, estStates, modelSpecs{iModel, 1});
                testCase.verifyGreaterThanOrEqual(elapsed, 0);
            end

            immCfg = tracking.single.dbt.ConfigIMM('numSteps', numSteps, 'dt', 1);
            imm = tracking.single.dbt.IMM(immCfg);
            x0 = [truthX(1, 1); truthX(2, 1); 0; truthX(4, 1); truthX(5, 1); 0];
            P0 = diag([100, 25, 25, 100, 25, 25]);
            [estStates, ~, modelProbs, elapsed] = imm.run(meas, x0, P0);
            testCase.verifySize(estStates, [6, numSteps]);
            localVerifyFinite(testCase, estStates, 'IMM');
            testCase.verifyGreaterThanOrEqual(elapsed, 0);
            testCase.verifyEqual(sum(modelProbs, 1), ones(1, numSteps), 'AbsTol', 1e-10);
        end

        function testPhdScenarioRegression(testCase)
            rng(testCase.RegressionSeed);
            model = tracking.multi.rfs.phd.Config();
            scenario = tracking.multi.rfs.phd.Scenario(model);
            scenario = scenario.generateTruth();
            truth = struct( ...
                'X', {scenario.X}, ...
                'N', scenario.N, ...
                'K', scenario.K, ...
                'track_list', {scenario.track_list}, ...
                'total_tracks', scenario.total_tracks);
            scenario = scenario.generateMeasurements(scenario);
            meas = struct('Z', {scenario.Z}, 'K', scenario.K);

            filters = {
                'ImmPhdFilter', tracking.multi.rfs.phd.ImmPhdFilter(model)
                'SimmPhdFilter', tracking.multi.rfs.phd.SimmPhdFilter(model)
            };

            for iFilter = 1:size(filters, 1)
                est = filters{iFilter, 2}.run(meas, truth);
                testCase.verifyEqual(numel(est.IMMX), meas.K, filters{iFilter, 1});
                testCase.verifyTrue(isfield(est, 'IMMN'));
                testCase.verifyEqual(numel(est.IMMN), meas.K);
            end
        end

        function testPointTargetRfsRegression(testCase)
            rng(testCase.RegressionSeed);
            config = localFilterConfig();
            measurements = {
                [1.0 2.0; 3.0 4.0]
                [1.1 2.1; 3.1 4.1]
            };
            truth = {
                [1.0 2.0; 3.0 4.0; 0 0; 0 0]
                [1.1 2.1; 3.1 4.1; 0 0; 0 0]
            };
            filters = {
                'GMPHD', @() tracking.multi.rfs.phd.GMPHD(config)
                'GMCPHD', @() tracking.multi.rfs.phd.GMCPHD(config)
                'PMB', @() tracking.multi.rfs.pmbm.PMB(config)
                'PMBM', @() tracking.multi.rfs.pmbm.PMBM(config)
                'CDGMPHD', @() tracking.multi.rfs.cd.CDGMPHD(config)
                'CDGMCPHD', @() tracking.multi.rfs.cd.CDGMCPHD(config)
                'CDPMBM', @() tracking.multi.rfs.cd.CDPMBM(config)
            };

            for iFilter = 1:size(filters, 1)
                estimates = localRunRfsFilter(filters{iFilter, 2}(), measurements);
                localVerifyEstimateSequence(testCase, estimates, 10, filters{iFilter, 1});
                [meanOspa, meanGospa] = localMultiMetrics(estimates, truth);
                testCase.verifyLessThanOrEqual(meanOspa, 100, filters{iFilter, 1});
                testCase.verifyLessThanOrEqual(meanGospa, 100, filters{iFilter, 1});
            end
        end

        function testTrajectoryRfsRegression(testCase)
            rng(testCase.RegressionSeed);
            config = localFilterConfig();
            measurements = [1.0 2.0; 3.0 4.0];
            filters = {
                'GMTPHD', @() tracking.multi.rfs.trajectory.phd.GMTPHD(config)
                'TPMB', @() tracking.multi.rfs.trajectory.pmb.TPMB(config)
                'TPMBM', @() tracking.multi.rfs.trajectory.pmbm.TPMBM(config)
                'TMBM', @() tracking.multi.rfs.trajectory.mbm.TMBM(config)
            };

            for iFilter = 1:size(filters, 1)
                filter = filters{iFilter, 2}();
                filter = filter.initialize();
                filter = filter.predict();
                filter = filter.update(measurements);
                estimate = filter.estimate();
                states = localCollectStates(estimate);
                localVerifyFinite(testCase, states, filters{iFilter, 1});
                testCase.verifyLessThanOrEqual(localCardinality(estimate), 10, filters{iFilter, 1});
            end
        end

        function testLabeledRfsRegression(testCase)
            configs = {
                'Joint-GLMB-GMS', tracking.multi.rfs.labeled.glmb.JointGlmbGms()
                'Joint-LMB-GMS', tracking.multi.rfs.labeled.lmb.JointLmbGms()
            };

            truth = cell(3, 1);
            for k = 1:3
                truth{k} = [k, k + 0.5; 0.5 * k, -0.25 * k; 0 0; 0 0];
            end

            for iFilter = 1:size(configs, 1)
                result = configs{iFilter, 2}.run(struct('numSteps', 3, 'seed', testCase.RegressionSeed));
                localValidateTrackingResult(testCase, result);
                for k = 1:numel(result.estimates)
                    estimate = result.estimates{k};
                    testCase.verifyTrue(isfield(estimate, 'labels'));
                    testCase.verifyTrue(isfield(estimate, 'scores'));
                    testCase.verifyEqual(localCardinality(estimate), 2, configs{iFilter, 1});
                end
                [~, meanGospa] = localMultiMetrics(result.estimates, truth);
                testCase.verifyTrue(isfinite(meanGospa), configs{iFilter, 1});
            end
        end

        function testExtendedTargetRegression(testCase)
            filters = {
                'GgiwFilter', tracking.extended.ggiw.GgiwFilter()
                'StarConvexTracker', tracking.extended.starconvex.StarConvexTracker()
                'ExtendedTargetPhdFilter', tracking.extended.phd.ExtendedTargetPhdFilter()
                'TargetPmbmFilter', tracking.extended.pmbm.TargetPmbmFilter()
                'TrajectoryPmbmFilter', tracking.extended.pmbm.TrajectoryPmbmFilter()
                'GgiwPmbmSmoother', tracking.extended.pmbm.smoothing.GgiwPmbmSmoother()
            };

            for iFilter = 1:size(filters, 1)
                result = filters{iFilter, 2}.run(struct('numSteps', 3, 'seed', testCase.RegressionSeed));
                localValidateTrackingResult(testCase, result);
                for k = 1:numel(result.estimates)
                    localVerifyExtendedEstimate(testCase, result.estimates{k}, filters{iFilter, 1});
                end
            end
        end

        function testRegressionRunnerCreatesSummary(testCase)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            summaryPath = fullfile(repoRoot, 'results', 'regression_summary.json');
            if isfile(summaryPath)
                delete(summaryPath);
            end

            results = runRegressionTests();
            testCase.verifyTrue(isfile(summaryPath));
            testCase.verifyTrue(all([results.Passed]));
            summary = jsondecode(fileread(summaryPath));
            testCase.verifyEqual(summary.seed, testCase.RegressionSeed);
            testCase.verifyGreaterThan(summary.totalTests, 0);
            testCase.verifyLessThan(summary.runtimeSeconds, testCase.RuntimeLimitSeconds);
        end
    end
end

function config = localFilterConfig()
    config = tracking.multi.rfs.core.FilterConfig();
    config.detectionProb = 0.9;
    config.survivalProb = 0.99;
    config.clutterRate = 1e-4;
    config.clutterIntensity = 1e-4;
    config.surveillanceArea = [1000, 1000];
    config.pruningThreshold = 1e-5;
    config.maxComponents = 50;
    config.gatingThreshold = 20;
    config.existenceThreshold = 1e-5;
    config.motionModel = struct('type', 'CV', 'F', eye(4), 'Q', eye(4));
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

function estimates = localRunRfsFilter(filter, measurements)
    filter = filter.initialize();
    estimates = cell(size(measurements));
    for k = 1:numel(measurements)
        if isa(filter, 'tracking.multi.rfs.cd.CDGMPHD') || ...
                isa(filter, 'tracking.multi.rfs.cd.CDGMCPHD') || ...
                isa(filter, 'tracking.multi.rfs.cd.CDPMBM')
            filter = filter.predict(1);
        else
            filter = filter.predict();
        end
        filter = filter.update(measurements{k});
        estimates{k} = filter.estimate();
    end
end

function localVerifyEstimateSequence(testCase, estimates, maxCardinality, label)
    for k = 1:numel(estimates)
        states = localCollectStates(estimates{k});
        localVerifyFinite(testCase, states, label);
        cardinality = localCardinality(estimates{k});
        testCase.verifyGreaterThanOrEqual(cardinality, 0, label);
        testCase.verifyLessThanOrEqual(cardinality, maxCardinality, label);
    end
end

function states = localCollectStates(estimate)
    if isempty(estimate)
        states = zeros(4, 0);
    elseif iscell(estimate)
        if isempty(estimate)
            states = zeros(4, 0);
        else
            states = localCollectStates(estimate{end});
        end
    elseif isstruct(estimate) && isfield(estimate, 'states')
        states = estimate.states;
    elseif isstruct(estimate) && isfield(estimate, 'trajectories') && ~isempty(estimate.trajectories)
        states = localCollectStates(estimate.trajectories);
    elseif isnumeric(estimate)
        states = estimate;
    else
        states = zeros(4, 0);
    end

    if isempty(states)
        states = zeros(4, 0);
    elseif isrow(states)
        states = states(:);
    end
    if size(states, 1) == 2
        states = [states; zeros(2, size(states, 2))];
    end
end

function cardinality = localCardinality(estimate)
    if isstruct(estimate) && isfield(estimate, 'cardinality')
        cardinality = double(estimate.cardinality);
    else
        cardinality = size(localCollectStates(estimate), 2);
    end
end

function value = localRmse(est, truth, dims)
    diff = est(dims, :) - truth(dims, :);
    value = mean(sqrt(sum(diff.^2, 1)));
end

function [meanOspa, meanGospa] = localMultiMetrics(estimates, truth)
    gospa = tracking.metrics.GOSPA('p', 2, 'c', 100, 'alpha', 2);
    ospaValues = zeros(1, numel(estimates));
    gospaValues = zeros(1, numel(estimates));
    for k = 1:numel(estimates)
        states = localCollectStates(estimates{k});
        truthStates = localCollectStates(truth{k});
        ospaValues(k) = tracking.metrics.OspaMetric.compute(states, truthStates, 100, 2);
        gospaValues(k) = gospa.compute(states, truthStates);
    end
    meanOspa = mean(ospaValues);
    meanGospa = mean(gospaValues);
end

function localValidateTrackingResult(testCase, result)
    requiredFields = {'metadata', 'truth', 'measurements', 'estimates', 'metrics', 'config'};
    testCase.verifyTrue(isstruct(result));
    for iField = 1:numel(requiredFields)
        testCase.verifyTrue(isfield(result, requiredFields{iField}), requiredFields{iField});
    end
    testCase.verifyTrue(isstruct(result.metadata));
    testCase.verifyTrue(iscell(result.estimates));
end

function localVerifyExtendedEstimate(testCase, estimate, label)
    testCase.verifyTrue(isstruct(estimate), label);
    for fieldName = {'states', 'covariances', 'labels', 'scores', 'extents', 'cardinality'}
        testCase.verifyTrue(isfield(estimate, fieldName{1}), label);
    end
    testCase.verifyEqual(estimate.cardinality, size(estimate.states, 2), label);
    localVerifyFinite(testCase, estimate.states, label);
    localVerifyFinite(testCase, estimate.covariances, label);
    localVerifyFinite(testCase, estimate.extents, label);
    for iExtent = 1:size(estimate.extents, 3)
        eigValues = eig((estimate.extents(:, :, iExtent) + estimate.extents(:, :, iExtent)') / 2);
        testCase.verifyGreaterThanOrEqual(min(eigValues), -1e-10, label);
    end
end

function localVerifyFinite(testCase, values, label)
    testCase.verifyTrue(all(isfinite(values(:))), label);
end
