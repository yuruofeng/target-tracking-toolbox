function result = createTrackingResult(metadata, truth, measurements, estimates, metrics, config)
% CREATETRACKINGRESULT  Create the shared TrackingResult struct.

    if nargin < 1 || isempty(metadata), metadata = struct(); end
    if nargin < 2, truth = []; end
    if nargin < 3, measurements = []; end
    if nargin < 4, estimates = []; end
    if nargin < 5 || isempty(metrics), metrics = struct(); end
    if nargin < 6 || isempty(config), config = struct(); end

    result = struct();
    result.metadata = metadata;
    result.truth = truth;
    result.measurements = measurements;
    result.estimates = estimates;
    result.metrics = metrics;
    result.config = config;
end
