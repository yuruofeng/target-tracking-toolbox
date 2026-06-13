function varargout = demo_extended_ggiw()
% DEMO_EXTENDED_GGIW  Run the GGIW-PHD extended-target adapter demo.

    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    resultsDir = fullfile(repoRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    config = tracking.io.loadConfig(fullfile(repoRoot, 'configs', 'extended', 'ggiw_default.json'));
    filter = tracking.extended.ggiw.GgiwFilter(config);
    result = filter.run(config);

    outputFile = fullfile(resultsDir, 'demo_extended_ggiw.mat');
    save(outputFile, 'result');
    fprintf('Saved GGIW-PHD adapter demo result to %s\n', outputFile);

    if nargout > 0
        varargout{1} = result;
    end
end
