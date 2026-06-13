function varargout = demo_extended_phd()
% DEMO_EXTENDED_PHD  Run the extended-target PHD adapter demo.

    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    resultsDir = fullfile(repoRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    config = tracking.io.loadConfig(fullfile(repoRoot, 'configs', 'extended', 'extended_phd_default.json'));
    filter = tracking.extended.phd.ExtendedTargetPhdFilter(config);
    result = filter.run(config);

    outputFile = fullfile(resultsDir, 'demo_extended_phd.mat');
    save(outputFile, 'result');
    fprintf('Saved extended-target PHD adapter demo result to %s\n', outputFile);

    if nargout > 0
        varargout{1} = result;
    end
end
