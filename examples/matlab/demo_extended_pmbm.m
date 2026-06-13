function varargout = demo_extended_pmbm()
% DEMO_EXTENDED_PMBM  Run extended-target PMBM adapter demos.

    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    resultsDir = fullfile(repoRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    config = tracking.io.loadConfig(fullfile(repoRoot, 'configs', 'extended', 'pmbm_default.json'));
    filters = {
        tracking.extended.pmbm.TargetPmbmFilter(config)
        tracking.extended.pmbm.TrajectoryPmbmFilter(config)
    };

    results = cell(numel(filters), 1);
    for iFilter = 1:numel(filters)
        results{iFilter} = filters{iFilter}.run();
    end

    outputFile = fullfile(resultsDir, 'demo_extended_pmbm.mat');
    save(outputFile, 'results');
    fprintf('Saved extended PMBM demo results to %s\n', outputFile);

    if nargout > 0
        varargout{1} = results;
    end
end
