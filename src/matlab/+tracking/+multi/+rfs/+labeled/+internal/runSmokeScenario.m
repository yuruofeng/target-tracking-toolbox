function result = runSmokeScenario(algorithmName, config)
% RUNSMOKESCENARIO  Run a small deterministic labeled-RFS adapter scenario.

    if nargin < 2 || isempty(config)
        config = struct();
    end
    if ~isfield(config, 'numSteps')
        config.numSteps = 3;
    end
    if ~isfield(config, 'seed')
        config.seed = 42;
    end

    rng(config.seed);
    measurements = cell(config.numSteps, 1);
    estimates = cell(config.numSteps, 1);
    truth = cell(config.numSteps, 1);

    for k = 1:config.numSteps
        z = [k, k + 0.5; 0.5 * k, -0.25 * k];
        measurements{k} = z;
        estimates{k} = tracking.multi.rfs.labeled.internal.measurementToEstimate(z, algorithmName(1));
        truth{k} = estimates{k}.states;
    end

    metadata = struct( ...
        'algorithm', algorithmName, ...
        'taskType', 'multi-target', ...
        'family', 'labeled-rfs', ...
        'backend', 'matlab', ...
        'seed', config.seed, ...
        'source', tracking.multi.rfs.labeled.internal.sourceInfo());
    metrics = struct('runtime', 0, 'numSteps', config.numSteps);
    result = tracking.core.createTrackingResult(metadata, truth, measurements, estimates, metrics, config);
end
