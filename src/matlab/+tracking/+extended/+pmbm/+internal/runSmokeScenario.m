function result = runSmokeScenario(algorithmName, config, sourceName)
% RUNSMOKESCENARIO  Run a small deterministic extended-PMBM adapter scenario.

    if nargin < 2 || isempty(config)
        config = struct();
    end
    if nargin < 3 || isempty(sourceName)
        sourceName = 'extended-target-pmbm';
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
    includeTrajectories = contains(algorithmName, 'Trajectory') || contains(algorithmName, 'Smoother');

    for k = 1:config.numSteps
        z = [k, k + 0.25; 0.4 * k, -0.2 * k];
        measurements{k} = z;
        estimates{k} = tracking.extended.pmbm.internal.measurementToEstimate(z, 'E', includeTrajectories);
        truth{k} = struct('states', estimates{k}.states, 'extents', estimates{k}.extents);
    end

    metadata = struct( ...
        'algorithm', algorithmName, ...
        'taskType', 'extended-target', ...
        'family', 'extended-pmbm', ...
        'backend', 'matlab', ...
        'seed', config.seed, ...
        'source', tracking.extended.pmbm.internal.sourceInfo(sourceName));
    metrics = struct('runtime', 0, 'numSteps', config.numSteps);
    result = tracking.core.createTrackingResult(metadata, truth, measurements, estimates, metrics, config);
end
