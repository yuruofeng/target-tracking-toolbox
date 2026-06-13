function estimate = measurementToEstimate(measurements, labelPrefix)
% MEASUREMENTTOESTIMATE  Convert point measurements to a labeled estimate.

    if nargin < 1 || isempty(measurements)
        measurements = zeros(2, 0);
    end
    if nargin < 2 || isempty(labelPrefix)
        labelPrefix = 'L';
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
    labels = cell(1, nTargets);
    scores = ones(1, nTargets);
    for iTarget = 1:nTargets
        covariances(:, :, iTarget) = eye(4);
        labels{iTarget} = sprintf('%s%d', labelPrefix, iTarget);
    end

    estimate = struct( ...
        'states', states, ...
        'covariances', covariances, ...
        'labels', {labels}, ...
        'scores', scores, ...
        'cardinality', nTargets);
end
