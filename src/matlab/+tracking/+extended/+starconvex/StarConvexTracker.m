classdef StarConvexTracker
% STARCONVEXTRACKER  Public adapter for the star-convex extended-target engine.

    properties
        Config struct = struct()
        State struct = struct()
        Engine = []
        Initialized (1, 1) logical = false
    end

    methods
        function obj = StarConvexTracker(config)
            if nargin > 0 && ~isempty(config)
                obj.Config = config;
            end
            obj.State = struct('time', 0, 'estimate', obj.emptyEstimate());
        end

        function obj = initialize(obj, initialCondition)
            if nargin < 2 || isempty(initialCondition)
                initialCondition = struct();
            end
            config = obj.mergeConfig(initialCondition);
            rng(obj.getConfigValue(config, 'seed', 20260612));

            numFourierCoeff = obj.getConfigValue(config, 'numFourierCoeff', 11);
            filterType = string(obj.getConfigValue(config, 'filterType', 'UKF'));
            manager = tracking.extended.utils.ConfigManager();
            manager.set('starconvex.scaleMean', obj.getConfigValue(config, 'scaleMean', 0.7));
            manager.set('starconvex.scaleVariance', obj.getConfigValue(config, 'scaleVariance', 0.08));

            state = obj.getConfigValue(config, 'state', [100; 100; 5; -8; 100; zeros(numFourierCoeff - 1, 1)]);
            covariance = obj.getConfigValue(config, 'covariance', ...
                diag([100, 100, 10, 10, ones(1, numFourierCoeff) * 0.1]));

            obj.Config = config;
            obj.Engine = tracking.extended.internal.StarConvexEngine(numFourierCoeff, filterType, manager);
            obj.Engine.initialize(state, covariance);
            obj.State = struct('time', 0, 'estimate', obj.standardizeEstimate());
            obj.Initialized = true;
        end

        function obj = predict(obj, stepContext)
            if nargin < 2 || isempty(stepContext)
                stepContext = struct();
            end
            obj = obj.ensureInitialized();
            model = obj.defaultModel(obj.Engine.numFourierCoeff);
            F = obj.getConfigValue(stepContext, 'F', model.F);
            Q = obj.getConfigValue(stepContext, 'Q', model.Q);
            obj.Engine.predict(F, Q);
            obj.State.time = obj.State.time + 1;
            obj.State.estimate = obj.standardizeEstimate();
        end

        function obj = update(obj, measurements, stepContext)
            if nargin < 3 || isempty(stepContext)
                stepContext = struct();
            end
            obj = obj.ensureInitialized();
            model = obj.defaultModel(obj.Engine.numFourierCoeff);
            measurementNoiseMean = obj.getConfigValue(stepContext, 'measurementNoiseMean', model.measurementNoiseMean);
            measurementNoiseCov = obj.getConfigValue(stepContext, 'measurementNoiseCov', model.measurementNoiseCov);
            normalized = obj.normalizeMeasurements(measurements);
            for iMeasurement = 1:size(normalized, 2)
                obj.Engine.update(normalized(:, iMeasurement), measurementNoiseMean, measurementNoiseCov);
            end
            obj.State.estimate = obj.standardizeEstimate();
        end

        function estimate = estimate(obj)
            if isfield(obj.State, 'estimate') && ~isempty(obj.State.estimate)
                estimate = obj.State.estimate;
            else
                estimate = obj.emptyEstimate();
            end
        end

        function result = run(obj, scenarioOrData, varargin)
            if nargin < 2 || isempty(scenarioOrData)
                scenarioOrData = struct();
            end
            config = obj.mergeConfig(scenarioOrData, varargin{:});
            numSteps = obj.getConfigValue(config, 'numSteps', 3);
            seed = obj.getConfigValue(config, 'seed', 20260612);
            rng(seed);

            timer = tic;
            runner = obj.initialize(config);
            measurements = cell(1, numSteps);
            estimates = cell(1, numSteps);
            truth = cell(1, numSteps);

            for k = 1:numSteps
                z = [100 + k, 100 + k + 0.3, 100 + k - 0.2; ...
                     100 - 0.5 * k, 100 - 0.5 * k + 0.2, 100 - 0.5 * k - 0.1];
                runner = runner.predict(struct('time', k));
                runner = runner.update(z, struct('time', k));
                measurements{k} = z;
                estimates{k} = runner.estimate();
                truth{k} = struct('states', [100 + k; 5; 100 - 0.5 * k; -8], 'extents', eye(2) * 100);
            end

            metadata = struct('algorithm', obj.getConfigValue(config, 'algorithm', 'Star-Convex-Tracker'), ...
                'taskType', 'extended-target', 'seed', seed, 'backend', 'matlab');
            metrics = struct('runtime', toc(timer), 'numSteps', numSteps);
            result = tracking.core.createTrackingResult(metadata, truth, measurements, ...
                estimates, metrics, config);
        end
    end

    methods (Access = private)
        function obj = ensureInitialized(obj)
            if ~obj.Initialized
                obj = obj.initialize(obj.Config);
            end
        end

        function model = defaultModel(~, numFourierCoeff)
            model = struct();
            model.F = blkdiag([eye(2), 10 * eye(2); zeros(2, 2), eye(2)], eye(numFourierCoeff));
            model.Q = blkdiag(diag([10, 10, 1, 1]), diag(ones(1, numFourierCoeff) * 0.1));
            model.measurementNoiseMean = [0.7; 0; 0];
            model.measurementNoiseCov = diag([0.01, 0.1, 0.1]);
        end

        function measurements = normalizeMeasurements(~, measurements)
            if nargin < 2 || isempty(measurements)
                measurements = zeros(2, 0);
            elseif isstruct(measurements) && isfield(measurements, 'points')
                measurements = measurements.points;
            elseif iscell(measurements)
                measurements = measurements{1};
            end
        end

        function estimate = standardizeEstimate(obj)
            states = [obj.Engine.getPosition(); obj.Engine.getVelocity()];
            estimate = obj.emptyEstimate(1);
            estimate.states = states;
            estimate.covariances(:, :, 1) = obj.makePsd(obj.Engine.covarianceMatrix(1:4, 1:4));
            estimate.labels = {'starconvex-1'};
            estimate.scores = 1;
            shape = obj.Engine.getShape(linspace(0, 2 * pi, 24));
            centered = shape - mean(shape, 2);
            extent = (centered * centered') / max(1, size(centered, 2) - 1);
            estimate.extents(:, :, 1) = obj.makePsd(extent + 1e-6 * eye(2));
        end

        function estimate = emptyEstimate(~, cardinality)
            if nargin < 2
                cardinality = 0;
            end
            estimate = struct();
            estimate.states = zeros(4, cardinality);
            estimate.covariances = repmat(eye(4), 1, 1, cardinality);
            estimate.labels = cell(1, cardinality);
            estimate.scores = zeros(1, cardinality);
            estimate.extents = repmat(eye(2), 1, 1, cardinality);
            estimate.cardinality = cardinality;
        end

        function matrix = makePsd(~, matrix)
            matrix = (matrix + matrix') / 2;
            [vectors, values] = eig(matrix);
            values = diag(max(diag(values), 0));
            matrix = (vectors * values * vectors');
            matrix = (matrix + matrix') / 2;
        end

        function config = mergeConfig(obj, scenarioOrData, varargin)
            config = obj.Config;
            if isstruct(scenarioOrData)
                names = fieldnames(scenarioOrData);
                for iName = 1:numel(names)
                    config.(names{iName}) = scenarioOrData.(names{iName});
                end
            end
            for iArg = 1:2:numel(varargin)
                config.(varargin{iArg}) = varargin{iArg + 1};
            end
        end

        function value = getConfigValue(~, config, name, defaultValue)
            if isstruct(config) && isfield(config, name)
                value = config.(name);
            else
                value = defaultValue;
            end
        end
    end
end
