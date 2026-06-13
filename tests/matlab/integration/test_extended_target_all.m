% TEST_EXTENDED_TARGET_ALL  Integration checks for extended-target adapters.

close all;
clc;

scriptPath = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(fileparts(scriptPath)));
addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));

timer = tic;
testResults = struct();
testResults.totalTests = 0;
testResults.passedTests = 0;
testResults.failedTests = 0;
testResults.testDetails = {};

fprintf('\n========================================\n');
fprintf('  Extended-Target Adapter Integration\n');
fprintf('  Test time: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('========================================\n\n');

adapterSpecs = {
    'GgiwFilter', tracking.extended.ggiw.GgiwFilter()
    'StarConvexTracker', tracking.extended.starconvex.StarConvexTracker()
    'ExtendedTargetPhdFilter', tracking.extended.phd.ExtendedTargetPhdFilter()
    'TargetPmbmFilter', tracking.extended.pmbm.TargetPmbmFilter()
    'TrajectoryPmbmFilter', tracking.extended.pmbm.TrajectoryPmbmFilter()
    'GgiwPmbmSmoother', tracking.extended.pmbm.smoothing.GgiwPmbmSmoother()
};

for iAdapter = 1:size(adapterSpecs, 1)
    testStart = tic;
    testName = adapterSpecs{iAdapter, 1};
    try
        result = adapterSpecs{iAdapter, 2}.run(struct('numSteps', 3, 'seed', 20260612 + iAdapter));
        localValidateTrackingResult(result);
        for k = 1:numel(result.estimates)
            localValidateExtendedEstimate(result.estimates{k});
        end
        testResults.passedTests = testResults.passedTests + 1;
        detail = struct('name', testName, 'status', 'passed', 'time', toc(testStart));
        fprintf('  [passed] %s\n', testName);
    catch exc
        testResults.failedTests = testResults.failedTests + 1;
        detail = struct('name', testName, 'status', 'failed', ...
            'time', toc(testStart), 'error', exc.message);
        fprintf('  [failed] %s: %s\n', testName, exc.message);
    end
    testResults.totalTests = testResults.totalTests + 1;
    testResults.testDetails{end + 1} = detail; %#ok<SAGROW>
end

testStart = tic;
try
    clutter = tracking.extended.ggiw.generateClutter(5e-6, [-200, 200, -200, 200], 3);
    assert(numel(clutter) == 3);
    generated = tracking.extended.ggiw.generateExtendedMeasurements([0; 0], 5, 0.99, eye(2), 3);
    assert(numel(generated) == 3);
    partitions = tracking.extended.phd.partitionMeasurementSet([0, 10; 0, 10], 50, 10);
    assert(partitions.numCells >= 1);
    testResults.passedTests = testResults.passedTests + 1;
    detail = struct('name', 'extended helpers', 'status', 'passed', 'time', toc(testStart));
    fprintf('  [passed] extended helpers\n');
catch exc
    testResults.failedTests = testResults.failedTests + 1;
    detail = struct('name', 'extended helpers', 'status', 'failed', ...
        'time', toc(testStart), 'error', exc.message);
    fprintf('  [failed] extended helpers: %s\n', exc.message);
end
testResults.totalTests = testResults.totalTests + 1;
testResults.testDetails{end + 1} = detail;

testResults.totalTime = toc(timer);
testResults.passRate = testResults.passedTests / testResults.totalTests * 100;

fprintf('\n========================================\n');
fprintf('  Extended Integration Summary\n');
fprintf('========================================\n');
fprintf('  Total: %d\n', testResults.totalTests);
fprintf('  Passed: %d\n', testResults.passedTests);
fprintf('  Failed: %d\n', testResults.failedTests);
fprintf('  Pass rate: %.1f%%\n', testResults.passRate);
fprintf('  Runtime: %.2f seconds\n', testResults.totalTime);
fprintf('========================================\n\n');

resultsDir = fullfile(repoRoot, 'results');
if ~isfolder(resultsDir)
    mkdir(resultsDir);
end
save(fullfile(resultsDir, 'test_results.mat'), 'testResults');

assert(testResults.failedTests == 0, 'Extended-target integration tests failed.');

function localValidateTrackingResult(result)
    requiredFields = {'metadata', 'truth', 'measurements', 'estimates', 'metrics', 'config'};
    assert(isstruct(result), 'TrackingResult must be a struct.');
    for iField = 1:numel(requiredFields)
        assert(isfield(result, requiredFields{iField}), ...
            'TrackingResult missing field: %s', requiredFields{iField});
    end
    assert(iscell(result.estimates), 'TrackingResult estimates must be a cell array.');
    assert(isfield(result.metadata, 'taskType'), 'metadata.taskType missing.');
    assert(strcmp(result.metadata.taskType, 'extended-target'), ...
        'metadata.taskType must be extended-target.');
end

function localValidateExtendedEstimate(estimate)
    requiredFields = {'states', 'covariances', 'labels', 'scores', 'extents', 'cardinality'};
    assert(isstruct(estimate), 'Estimate must be a struct.');
    for iField = 1:numel(requiredFields)
        assert(isfield(estimate, requiredFields{iField}), ...
            'Estimate missing field: %s', requiredFields{iField});
    end
    assert(all(isfinite(estimate.states(:))), 'Estimate states must be finite.');
    assert(all(isfinite(estimate.covariances(:))), 'Estimate covariances must be finite.');
    assert(all(isfinite(estimate.extents(:))), 'Estimate extents must be finite.');
    assert(estimate.cardinality == size(estimate.states, 2), ...
        'Estimate cardinality must match state count.');
    for iExtent = 1:size(estimate.extents, 3)
        extent = estimate.extents(:, :, iExtent);
        assert(size(extent, 1) == size(extent, 2), 'Extent must be square.');
        eigenvalues = eig((extent + extent') / 2);
        assert(min(eigenvalues) >= -1e-10, 'Extent must be positive semidefinite.');
    end
end
