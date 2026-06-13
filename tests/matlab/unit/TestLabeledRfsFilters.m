classdef TestLabeledRfsFilters < matlab.unittest.TestCase
% TESTLABELEDRFSFILTERS  Smoke tests for labeled RFS adapters.

    methods (TestClassSetup)
        function addSourcePath(~)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
        end
    end

    methods (Test)
        function testLabeledRfsClassesExist(testCase)
            testCase.verifyEqual(exist('tracking.multi.rfs.labeled.glmb.JointGlmbGms', 'class'), 8);
            testCase.verifyEqual(exist('tracking.multi.rfs.labeled.lmb.JointLmbGms', 'class'), 8);
        end

        function testGlmbAdapterImplementsTrackingInterface(testCase)
            filter = tracking.multi.rfs.labeled.glmb.JointGlmbGms();
            filter = filter.initialize(struct('seed', 7));
            filter = filter.predict(struct('time', 1));
            filter = filter.update([1; 2], struct('time', 1));
            estimate = filter.estimate();

            testCase.verifyEstimateStruct(estimate);
        end

        function testLmbAdapterImplementsTrackingInterface(testCase)
            filter = tracking.multi.rfs.labeled.lmb.JointLmbGms();
            filter = filter.initialize(struct('seed', 11));
            filter = filter.predict(struct('time', 1));
            filter = filter.update([3; 4], struct('time', 1));
            estimate = filter.estimate();

            testCase.verifyEstimateStruct(estimate);
        end

        function testLabeledRunReturnsTrackingResult(testCase)
            filter = tracking.multi.rfs.labeled.glmb.JointGlmbGms();
            result = filter.run(struct('numSteps', 2, 'seed', 13));

            testCase.verifyTrackingResult(result);
            testCase.verifyEqual(result.metadata.taskType, 'multi-target');
            testCase.verifyEqual(result.metadata.family, 'labeled-rfs');
        end
    end

    methods
        function verifyEstimateStruct(testCase, estimate)
            testCase.verifyTrue(isstruct(estimate));
            testCase.verifyTrue(isfield(estimate, 'states'));
            testCase.verifyTrue(isfield(estimate, 'covariances'));
            testCase.verifyTrue(isfield(estimate, 'labels'));
            testCase.verifyTrue(isfield(estimate, 'scores'));
        end

        function verifyTrackingResult(testCase, result)
            requiredFields = {'metadata', 'truth', 'measurements', 'estimates', 'metrics', 'config'};
            for iField = 1:numel(requiredFields)
                testCase.verifyTrue(isfield(result, requiredFields{iField}), requiredFields{iField});
            end
        end
    end
end
