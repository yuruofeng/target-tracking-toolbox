classdef TestDbtFilters < matlab.unittest.TestCase
% TESTDBTFILTERS  Unit tests for DBT filter implementations.
%   Tests EKF, UKF, CKF, and Particle Filter for correctness.
%
%   Run tests:
%       results = runtests('TestDbtFilters')
%       results = TestDbtFilters.run()

    properties
        config
        truthX
        meas
        x0
        P0
    end

    methods (TestMethodSetup)

        function setupScenario(testCase)
            unitFolder = fileparts(mfilename('fullpath'));
            repoRoot = fileparts(fileparts(fileparts(unitFolder)));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
            
            testCase.config = tracking.single.dbt.Config('numSteps', 50, 'numParticles', 500);
            scenario = tracking.single.dbt.Scenario(testCase.config);
            [testCase.truthX, testCase.meas] = scenario.generate();
            testCase.x0 = testCase.config.initState;
            testCase.P0 = blkdiag(10*eye(4), pi/90);
        end

    end

    methods (Test)

        function testEKFrun(testCase)
            ekf = tracking.single.dbt.EKF(testCase.config);
            [estStates, estCovars, elapsed] = ekf.run(testCase.meas, testCase.x0, testCase.P0);

            testCase.assertEqual(size(estStates, 2), testCase.config.numSteps);
            testCase.assertEqual(size(estCovars, 3), testCase.config.numSteps);
            testCase.verifyGreaterThan(elapsed, 0);

            posIdx = [1, 3];
            posRmse = mean(sqrt(sum((estStates(posIdx, :) - testCase.truthX(posIdx, :)).^2, 1)));
            testCase.verifyLessThan(posRmse, 50, 'EKF position RMSE should be reasonable');
        end

        function testUKFrun(testCase)
            ukf = tracking.single.dbt.UKF(testCase.config);
            [estStates, estCovars, elapsed] = ukf.run(testCase.meas, testCase.x0, testCase.P0);

            testCase.assertEqual(size(estStates, 2), testCase.config.numSteps);
            testCase.assertEqual(size(estCovars, 3), testCase.config.numSteps);
            testCase.verifyGreaterThan(elapsed, 0);

            posIdx = [1, 3];
            posRmse = mean(sqrt(sum((estStates(posIdx, :) - testCase.truthX(posIdx, :)).^2, 1)));
            testCase.verifyLessThan(posRmse, 50, 'UKF position RMSE should be reasonable');
        end

        function testCKFrun(testCase)
            ckf = tracking.single.dbt.CKF(testCase.config);
            [estStates, estCovars, elapsed] = ckf.run(testCase.meas, testCase.x0, testCase.P0);

            testCase.assertEqual(size(estStates, 2), testCase.config.numSteps);
            testCase.assertEqual(size(estCovars, 3), testCase.config.numSteps);
            testCase.verifyGreaterThan(elapsed, 0);

            posIdx = [1, 3];
            posRmse = mean(sqrt(sum((estStates(posIdx, :) - testCase.truthX(posIdx, :)).^2, 1)));
            testCase.verifyLessThan(posRmse, 50, 'CKF position RMSE should be reasonable');
        end

        function testParticleFilterRun(testCase)
            pf = tracking.single.dbt.ParticleFilter(testCase.config);
            [estStates, estCovars, elapsed] = pf.run(testCase.meas, testCase.x0, testCase.P0);

            testCase.assertEqual(size(estStates, 2), testCase.config.numSteps);
            testCase.assertEqual(size(estCovars, 3), testCase.config.numSteps);
            testCase.verifyGreaterThan(elapsed, 0);

            posIdx = [1, 3];
            posRmse = mean(sqrt(sum((estStates(posIdx, :) - testCase.truthX(posIdx, :)).^2, 1)));
            testCase.verifyLessThan(posRmse, 150, 'PF position RMSE should be reasonable');
        end

        function testConfigDefaultValues(testCase)
            cfg = tracking.single.dbt.Config();
            testCase.assertEqual(cfg.numSteps, 100);
            testCase.assertEqual(cfg.dt, 1);
            testCase.assertEqual(cfg.stateDim, 5);
            testCase.assertEqual(cfg.numParticles, 500);
        end

        function testConfigCustomValues(testCase)
            cfg = tracking.single.dbt.Config('numSteps', 200, 'dt', 0.5);
            testCase.assertEqual(cfg.numSteps, 200);
            testCase.assertEqual(cfg.dt, 0.5);
        end

        function testScenarioGeneration(testCase)
            cfg = tracking.single.dbt.Config('numSteps', 30);
            scenario = tracking.single.dbt.Scenario(cfg);
            [truthX, meas] = scenario.generate();

            testCase.assertEqual(size(truthX, 2), 30);
            testCase.assertEqual(size(meas, 2), 30);
            testCase.assertEqual(size(meas, 1), 2);
        end

        function testMeasurementModel(testCase)
            x = [10; 2; 5; 1; 0.02];
            z = tracking.models.MeasurementModel.ctMeasFunc(x);
            
            testCase.assertEqual(length(z), 2);
            testCase.verifyEqual(z(1), atan2(5, 10), 'AbsTol', 1e-10);
            testCase.verifyEqual(z(2), sqrt(125), 'AbsTol', 1e-10);
        end

        function testCtDynamicMatrix(testCase)
            x = [10; 2; 5; 1; 0];
            F = tracking.models.MeasurementModel.ctDynamicMatrix(1, x);
            
            testCase.assertEqual(size(F), [5, 5]);
            testCase.verifyEqual(F(1, 2), 1, 'AbsTol', 1e-10);
            testCase.verifyEqual(F(3, 4), 1, 'AbsTol', 1e-10);
        end

        function testFilterUtilsResample(testCase)
            weights = rand(100, 1);
            weights = weights / sum(weights);
            idx = tracking.core.FilterUtils.systematicResample(weights);
            
            testCase.assertEqual(length(idx), 100);
            testCase.verifyTrue(all(idx >= 1 & idx <= 100));
        end

    end
end
