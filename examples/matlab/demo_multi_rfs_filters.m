function varargout = demo_multi_rfs_filters()
% DEMO_MULTI_RFS_FILTERS  Run migrated multi-target RFS filters.

    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    resultsDir = fullfile(repoRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    rng(42);
    config = localFilterConfig();
    measurements = {
        [1.0 2.0; 3.0 4.0]
        [1.2 2.2; 3.1 4.1]
        [1.4 2.4; 3.2 4.2]
    };

    algorithms = {
        'GMPHD', @() tracking.multi.rfs.phd.GMPHD(config)
        'GMCPHD', @() tracking.multi.rfs.phd.GMCPHD(config)
        'PMB', @() tracking.multi.rfs.pmbm.PMB(config)
        'PMBM', @() tracking.multi.rfs.pmbm.PMBM(config)
        'CDGMPHD', @() tracking.multi.rfs.cd.CDGMPHD(config)
        'CDGMCPHD', @() tracking.multi.rfs.cd.CDGMCPHD(config)
        'CDPMBM', @() tracking.multi.rfs.cd.CDPMBM(config)
    };

    results = cell(size(algorithms, 1), 1);
    for iAlgorithm = 1:size(algorithms, 1)
        algorithmName = algorithms{iAlgorithm, 1};
        filter = algorithms{iAlgorithm, 2}();
        [estimates, runtime] = localRunFilter(filter, measurements);

        metadata = struct( ...
            'algorithm', algorithmName, ...
            'task_type', 'multi-target-rfs', ...
            'backend', 'matlab', ...
            'seed', 42);
        metrics = struct('runtime', runtime);
        results{iAlgorithm} = tracking.core.createTrackingResult( ...
            metadata, [], measurements, estimates, metrics, config.toStruct());
    end

    outputFile = fullfile(resultsDir, 'demo_multi_rfs_filters.mat');
    save(outputFile, 'results');
    fprintf('Saved multi-target RFS demo results to %s\n', outputFile);

    if nargout > 0
        varargout{1} = results;
    end
end

function [estimates, runtime] = localRunFilter(filter, measurements)
    filter = filter.initialize();
    estimates = cell(size(measurements));
    timer = tic;
    for k = 1:numel(measurements)
        if isa(filter, 'tracking.multi.rfs.cd.CDGMPHD') || ...
                isa(filter, 'tracking.multi.rfs.cd.CDGMCPHD') || ...
                isa(filter, 'tracking.multi.rfs.cd.CDPMBM')
            filter = filter.predict(1);
        else
            filter = filter.predict();
        end
        filter = filter.update(measurements{k});
        estimates{k} = filter.estimate();
    end
    runtime = toc(timer);
end

function config = localFilterConfig()
    config = tracking.multi.rfs.core.FilterConfig();
    config.detectionProb = 0.9;
    config.survivalProb = 0.99;
    config.clutterRate = 1e-4;
    config.clutterIntensity = 1e-4;
    config.surveillanceArea = [1000, 1000];
    config.pruningThreshold = 1e-5;
    config.maxComponents = 50;
    config.gatingThreshold = 20;
    config.existenceThreshold = 1e-5;
    config.motionModel = struct('type', 'CV', 'F', eye(4), 'Q', eye(4));
    config.measurementModel = struct( ...
        'type', 'Linear', ...
        'H', [1 0 0 0; 0 1 0 0], ...
        'R', eye(2));
    config.birthModel = struct( ...
        'type', 'Poisson', ...
        'intensity', 0.005, ...
        'means', zeros(4, 1), ...
        'covs', eye(4), ...
        'weights', 1);
    config.extraParams.totalTimeSteps = 10;
end
