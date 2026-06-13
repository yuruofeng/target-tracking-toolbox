function varargout = demo_multi_filter_comparison()
% DEMO_MULTI_FILTER_COMPARISON  Compare migrated multi-target filters.

    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repoRoot, 'src', 'matlab')));
    resultsDir = fullfile(repoRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    config = localFilterConfig();
    measurements = {
        [1.0 2.0; 3.0 4.0]
        [1.1 2.1; 3.1 4.1]
    };
    truth = {
        [1.0 2.0; 3.0 4.0; 0 0; 0 0]
        [1.1 2.1; 3.1 4.1; 0 0; 0 0]
    };

    algorithms = {
        'GMPHD', @() tracking.multi.rfs.phd.GMPHD(config)
        'PMB', @() tracking.multi.rfs.pmbm.PMB(config)
        'PMBM', @() tracking.multi.rfs.pmbm.PMBM(config)
        'CDGMPHD', @() tracking.multi.rfs.cd.CDGMPHD(config)
    };

    gospa = tracking.metrics.GOSPA('p', 2, 'c', 100, 'alpha', 2);
    summary = struct('algorithm', {}, 'runtime', {}, 'meanGospa', {}, 'meanCardinality', {});
    results = cell(size(algorithms, 1), 1);

    for iAlgorithm = 1:size(algorithms, 1)
        algorithmName = algorithms{iAlgorithm, 1};
        filter = algorithms{iAlgorithm, 2}();
        [estimates, runtime] = localRunFilter(filter, measurements);

        gospaValues = zeros(1, numel(estimates));
        cardinalities = zeros(1, numel(estimates));
        for k = 1:numel(estimates)
            states = localEstimateStates(estimates{k});
            cardinalities(k) = size(states, 2);
            gospaValues(k) = gospa.compute(states, truth{k});
        end

        metrics = struct( ...
            'runtime', runtime, ...
            'mean_gospa', mean(gospaValues), ...
            'mean_cardinality', mean(cardinalities));
        metadata = struct( ...
            'algorithm', algorithmName, ...
            'task_type', 'multi-target-comparison', ...
            'backend', 'matlab');
        results{iAlgorithm} = tracking.core.createTrackingResult( ...
            metadata, truth, measurements, estimates, metrics, config.toStruct());

        summary(iAlgorithm).algorithm = algorithmName; %#ok<AGROW>
        summary(iAlgorithm).runtime = runtime;
        summary(iAlgorithm).meanGospa = metrics.mean_gospa;
        summary(iAlgorithm).meanCardinality = metrics.mean_cardinality;
    end

    outputFile = fullfile(resultsDir, 'demo_multi_filter_comparison.mat');
    save(outputFile, 'results', 'summary');
    fprintf('Saved multi-filter comparison results to %s\n', outputFile);

    if nargout > 0
        varargout{1} = results;
        varargout{2} = summary;
    end
end

function [estimates, runtime] = localRunFilter(filter, measurements)
    filter = filter.initialize();
    estimates = cell(size(measurements));
    timer = tic;
    for k = 1:numel(measurements)
        if isa(filter, 'tracking.multi.rfs.cd.CDGMPHD')
            filter = filter.predict(1);
        else
            filter = filter.predict();
        end
        filter = filter.update(measurements{k});
        estimates{k} = filter.estimate();
    end
    runtime = toc(timer);
end

function states = localEstimateStates(estimate)
    if isstruct(estimate) && isfield(estimate, 'states')
        states = estimate.states;
    elseif isnumeric(estimate)
        states = estimate;
    else
        states = zeros(4, 0);
    end

    if isempty(states)
        states = zeros(4, 0);
    elseif size(states, 1) == 2
        states = [states; zeros(2, size(states, 2))];
    end
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
