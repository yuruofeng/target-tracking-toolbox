function results = runRegressionTests()
% RUNREGRESSIONTESTS Run deterministic numerical regression tests.

    repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));

    testFile = fullfile(repoRoot, 'tests', 'matlab', 'regression', ...
        'TestAlgorithmNumericalRegression.m');
    suite = matlab.unittest.TestSuite.fromFile(testFile);
    suiteNames = string({suite.Name});
    suite = suite(~contains(suiteNames, 'testRegressionRunnerCreatesSummary'));

    timer = tic;
    results = run(suite);
    runtimeSeconds = toc(timer);

    summary = struct();
    summary.seed = 20260612;
    summary.totalTests = numel(results);
    summary.passed = sum([results.Passed]);
    summary.failed = sum([results.Failed]);
    summary.incomplete = sum([results.Incomplete]);
    summary.runtimeSeconds = runtimeSeconds;
    summary.generatedAt = datestr(now, 30);

    resultsDir = fullfile(repoRoot, 'results');
    if ~isfolder(resultsDir)
        mkdir(resultsDir);
    end

    summaryPath = fullfile(resultsDir, 'regression_summary.json');
    fileId = fopen(summaryPath, 'w');
    assert(fileId > 0, 'Failed to create regression summary: %s', summaryPath);
    cleanup = onCleanup(@() fclose(fileId));
    fprintf(fileId, '%s', jsonencode(summary));
end
