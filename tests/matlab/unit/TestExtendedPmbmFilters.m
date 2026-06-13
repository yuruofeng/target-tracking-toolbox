classdef TestExtendedPmbmFilters < matlab.unittest.TestCase
% TESTEXTENDEDPMBMFILTERS  Smoke tests for extended PMBM adapters.

    methods (TestClassSetup)
        function addSourcePath(~)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
        end
    end

    methods (Test)
        function testExtendedPmbmClassesExist(testCase)
            testCase.verifyEqual(exist('tracking.extended.pmbm.TargetPmbmFilter', 'class'), 8);
            testCase.verifyEqual(exist('tracking.extended.pmbm.TrajectoryPmbmFilter', 'class'), 8);
            testCase.verifyEqual(exist('tracking.extended.pmbm.smoothing.GgiwPmbmSmoother', 'class'), 8);
        end

        function testTargetPmbmImplementsTrackingInterface(testCase)
            filter = tracking.extended.pmbm.TargetPmbmFilter();
            filter = filter.initialize(struct('seed', 3));
            filter = filter.predict(struct('time', 1));
            filter = filter.update(testCase.simpleMeasurements(), struct('time', 1));
            estimate = filter.estimate();

            testCase.verifyExtendedEstimate(estimate);
        end

        function testTrajectoryPmbmImplementsTrackingInterface(testCase)
            filter = tracking.extended.pmbm.TrajectoryPmbmFilter();
            filter = filter.initialize(struct('seed', 5));
            filter = filter.predict(struct('time', 1));
            filter = filter.update(testCase.simpleMeasurements(), struct('time', 1));
            estimate = filter.estimate();

            testCase.verifyExtendedEstimate(estimate);
            testCase.verifyTrue(isfield(estimate, 'trajectories'));
        end

        function testSmootherRunReturnsTrackingResult(testCase)
            smoother = tracking.extended.pmbm.smoothing.GgiwPmbmSmoother();
            result = smoother.run(struct('numSteps', 2, 'seed', 17));

            testCase.verifyTrackingResult(result);
            testCase.verifyEqual(result.metadata.taskType, 'extended-target');
            testCase.verifyEqual(result.metadata.algorithm, 'GGIW-PMBM-Smoother');
        end
    end

    methods
        function measurements = simpleMeasurements(~)
            measurements = [1.0 2.0; 0.5 -0.25];
        end

        function verifyExtendedEstimate(testCase, estimate)
            testCase.verifyTrue(isstruct(estimate));
            testCase.verifyTrue(isfield(estimate, 'states'));
            testCase.verifyTrue(isfield(estimate, 'covariances'));
            testCase.verifyTrue(isfield(estimate, 'labels'));
            testCase.verifyTrue(isfield(estimate, 'extents'));
        end

        function verifyTrackingResult(testCase, result)
            requiredFields = {'metadata', 'truth', 'measurements', 'estimates', 'metrics', 'config'};
            for iField = 1:numel(requiredFields)
                testCase.verifyTrue(isfield(result, requiredFields{iField}), requiredFields{iField});
            end
        end
    end
end
