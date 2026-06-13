function varargout = demo_extended_pmbm_smoothing()
% DEMO_EXTENDED_PMBM_SMOOTHING  Run the GGIW-PMBM smoothing adapter demo.

    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    resultsDir = fullfile(repoRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    config = tracking.io.loadConfig(fullfile(repoRoot, 'configs', 'extended', 'ggiw_pmbm_smoothing_default.json'));
    smoother = tracking.extended.pmbm.smoothing.GgiwPmbmSmoother(config);
    result = smoother.run();

    outputFile = fullfile(resultsDir, 'demo_extended_pmbm_smoothing.mat');
    save(outputFile, 'result');
    fprintf('Saved GGIW-PMBM smoothing demo result to %s\n', outputFile);

    if nargout > 0
        varargout{1} = result;
    end
end
