classdef TestExtendedAdapters < matlab.unittest.TestCase
% TESTEXTENDEDADAPTERS  Public adapter contract for extended-target filters.

    methods (TestClassSetup)
        function addSourcePath(~)
            repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
        end
    end

    methods (Test)
        function testExtendedAdaptersExposeUnifiedWorkflow(testCase)
            adapters = testCase.adapterRegistry();
            measurements = [1.0, 2.0, 3.0; 0.5, 0.25, -0.25];

            for iAdapter = 1:size(adapters, 1)
                adapter = adapters{iAdapter, 2}();
                adapter = adapter.initialize(struct('seed', 20260612));
                adapter = adapter.predict(struct('time', 1));
                adapter = adapter.update(measurements, struct('time', 1));
                estimate = adapter.estimate();

                testCase.verifyExtendedEstimate(estimate, adapters{iAdapter, 1});
            end
        end

        function testExtendedAdaptersRunTrackingResult(testCase)
            adapters = testCase.adapterRegistry();

            for iAdapter = 1:size(adapters, 1)
                adapter = adapters{iAdapter, 2}();
                result = adapter.run(struct('numSteps', 3, 'seed', 20260612));

                testCase.verifyTrackingResult(result, adapters{iAdapter, 1});
                testCase.verifyEqual(numel(result.estimates), 3, adapters{iAdapter, 1});
                for k = 1:numel(result.estimates)
                    testCase.verifyExtendedEstimate(result.estimates{k}, adapters{iAdapter, 1});
                end
            end
        end

        function testInternalEnginesExistButAreNotPublicAdapters(testCase)
            testCase.verifyEqual(exist('tracking.extended.internal.GgiwEngine', 'class'), 8);
            testCase.verifyEqual(exist('tracking.extended.internal.StarConvexEngine', 'class'), 8);
            testCase.verifyEqual(exist('tracking.extended.internal.ExtendedTargetPhdEngine', 'class'), 8);
        end
    end

    methods
        function adapters = adapterRegistry(~)
            adapters = {
                'GGIW-PHD', @() tracking.extended.ggiw.GgiwFilter()
                'Star-Convex', @() tracking.extended.starconvex.StarConvexTracker()
                'Extended-PHD', @() tracking.extended.phd.ExtendedTargetPhdFilter()
            };
        end

        function verifyTrackingResult(testCase, result, label)
            requiredFields = {'metadata', 'truth', 'measurements', 'estimates', 'metrics', 'config'};
            testCase.verifyTrue(isstruct(result), label);
            for iField = 1:numel(requiredFields)
                testCase.verifyTrue(isfield(result, requiredFields{iField}), ...
                    sprintf('%s missing %s', label, requiredFields{iField}));
            end
            testCase.verifyTrue(iscell(result.estimates), label);
            testCase.verifyTrue(isfield(result.metadata, 'algorithm'), label);
            testCase.verifyTrue(isfield(result.metadata, 'taskType'), label);
            testCase.verifyEqual(result.metadata.taskType, 'extended-target', label);
        end

        function verifyExtendedEstimate(testCase, estimate, label)
            requiredFields = {'states', 'covariances', 'labels', 'scores', 'extents', 'cardinality'};
            testCase.verifyTrue(isstruct(estimate), label);
            for iField = 1:numel(requiredFields)
                testCase.verifyTrue(isfield(estimate, requiredFields{iField}), ...
                    sprintf('%s missing %s', label, requiredFields{iField}));
            end
            testCase.verifyEqual(estimate.cardinality, size(estimate.states, 2), label);
            testCase.verifyEqual(size(estimate.covariances, 3), estimate.cardinality, label);
            testCase.verifyEqual(size(estimate.extents, 3), estimate.cardinality, label);
            testCase.verifyEqual(numel(estimate.labels), estimate.cardinality, label);
            testCase.verifyEqual(numel(estimate.scores), estimate.cardinality, label);
            testCase.verifyTrue(all(isfinite(estimate.states(:))), label);
            testCase.verifyTrue(all(isfinite(estimate.covariances(:))), label);
            testCase.verifyTrue(all(isfinite(estimate.extents(:))), label);

            for iExtent = 1:size(estimate.extents, 3)
                extent = estimate.extents(:, :, iExtent);
                testCase.verifyEqual(size(extent, 1), size(extent, 2), label);
                eigenvalues = eig((extent + extent') / 2);
                testCase.verifyGreaterThanOrEqual(min(eigenvalues), -1e-10, label);
            end
        end
    end
end
