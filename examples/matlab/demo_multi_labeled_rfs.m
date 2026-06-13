function varargout = demo_multi_labeled_rfs()
% DEMO_MULTI_LABELED_RFS  Run labeled RFS GLMB/LMB adapter demos.

    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    resultsDir = fullfile(repoRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    glmbConfig = tracking.io.loadConfig(fullfile(repoRoot, 'configs', 'multi', 'glmb_gms_default.json'));
    lmbConfig = tracking.io.loadConfig(fullfile(repoRoot, 'configs', 'multi', 'lmb_gms_default.json'));

    filters = {
        tracking.multi.rfs.labeled.glmb.JointGlmbGms(glmbConfig)
        tracking.multi.rfs.labeled.lmb.JointLmbGms(lmbConfig)
    };

    results = cell(numel(filters), 1);
    for iFilter = 1:numel(filters)
        results{iFilter} = filters{iFilter}.run();
    end

    outputFile = fullfile(resultsDir, 'demo_multi_labeled_rfs.mat');
    save(outputFile, 'results');
    fprintf('Saved labeled RFS demo results to %s\n', outputFile);

    if nargout > 0
        varargout{1} = results;
    end
end
