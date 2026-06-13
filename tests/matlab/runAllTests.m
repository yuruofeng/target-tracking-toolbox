function varargout = runAllTests()
% RUNALLTESTS  Execute the MATLAB test suite for Target Tracking Toolbox.

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('   Target Tracking Toolbox - MATLAB Test Runner\n');
    fprintf('============================================================\n\n');

    testRoot = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(fileparts(testRoot));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    addpath(fullfile(testRoot, 'unit'));

    testFiles = {
        fullfile(testRoot, 'unit', 'TestPackageStructure.m')
        fullfile(testRoot, 'unit', 'TestDbtFilters.m')
        fullfile(testRoot, 'unit', 'TestTbdAlgorithms.m')
        fullfile(testRoot, 'unit', 'TestVisualization.m')
        fullfile(testRoot, 'unit', 'TestManeuverTracking.m')
        fullfile(testRoot, 'unit', 'TestPhdFilter.m')
        fullfile(testRoot, 'unit', 'TestMultiPackageStructure.m')
        fullfile(testRoot, 'unit', 'TestMultiAssociationMetrics.m')
        fullfile(testRoot, 'unit', 'TestMultiRfsFilters.m')
    };

    allResults = matlab.unittest.TestResult.empty;
    for iTest = 1:numel(testFiles)
        [~, testName] = fileparts(testFiles{iTest});
        fprintf('Running %s...\n', testName);
        try
            result = runtests(testFiles{iTest});
            allResults = [allResults, result]; %#ok<AGROW>
            fprintf('  Passed: %d | Failed: %d\n\n', ...
                sum([result.Passed]), sum([result.Failed]));
        catch err
            fprintf('  Error: %s\n\n', err.message);
            rethrow(err);
        end
    end

    totalPassed = sum([allResults.Passed]);
    totalFailed = sum([allResults.Failed]);
    totalTests = numel(allResults);

    fprintf('============================================================\n');
    fprintf('   Test Summary\n');
    fprintf('============================================================\n');
    fprintf('Total Tests: %d\n', totalTests);
    fprintf('Passed: %d\n', totalPassed);
    fprintf('Failed: %d\n', totalFailed);
    fprintf('============================================================\n\n');

    if totalFailed > 0
        fprintf('Failed Tests:\n');
        for iResult = 1:numel(allResults)
            if ~allResults(iResult).Passed
                fprintf('  - %s\n', allResults(iResult).Name);
            end
        end
        fprintf('\n');
    end

    if nargout > 0
        varargout{1} = allResults;
    end
end
