function estimate = measurementToEstimate(measurements, labelPrefix, includeTrajectories)
% MEASUREMENTTOESTIMATE  Convert measurements to an extended-target estimate.

    if nargin < 1 || isempty(measurements)
        measurements = zeros(2, 0);
    end
    if nargin < 2 || isempty(labelPrefix)
        labelPrefix = 'E';
    end
    if nargin < 3
        includeTrajectories = false;
    end

    if iscell(measurements)
        measurements = measurements{end};
    end
    if isrow(measurements)
        measurements = measurements(:);
    end
    if size(measurements, 1) < 2
        measurements = [measurements; zeros(2 - size(measurements, 1), size(measurements, 2))];
    end

    nTargets = size(measurements, 2);
    states = zeros(4, nTargets);
    states(1, :) = measurements(1, :);
    states(3, :) = measurements(2, :);

    covariances = zeros(4, 4, nTargets);
    extents = zeros(2, 2, nTargets);
    labels = cell(1, nTargets);
    scores = ones(1, nTargets);
    trajectories = cell(1, nTargets);
    for iTarget = 1:nTargets
        covariances(:, :, iTarget) = eye(4);
        extents(:, :, iTarget) = eye(2);
        labels{iTarget} = sprintf('%s%d', labelPrefix, iTarget);
        trajectories{iTarget} = states(:, iTarget);
    end

    estimate = struct( ...
        'states', states, ...
        'covariances', covariances, ...
        'labels', {labels}, ...
        'scores', scores, ...
        'extents', extents, ...
        'cardinality', nTargets);
    if includeTrajectories
        estimate.trajectories = trajectories;
    end
end
