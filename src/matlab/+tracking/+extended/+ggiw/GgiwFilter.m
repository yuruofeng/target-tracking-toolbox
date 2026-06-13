classdef GgiwFilter
% GGIWFILTER  Public adapter for the extended-target GGIW-PHD engine.

    properties
        Config struct = struct()
        State struct = struct()
        Engine = []
        Initialized (1, 1) logical = false
    end

    methods
        function obj = GgiwFilter(config)
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

            manager = tracking.extended.utils.ConfigManager();
            manager.set('tracking.birthWeight', obj.getConfigValue(config, 'birthWeight', 0.1));
            manager.set('tracking.birthMean', obj.getConfigValue(config, 'birthMean', [0; 0; 0; 0]));
            manager.set('tracking.birthCov', obj.getConfigValue(config, 'birthCov', diag([100, 10, 100, 10])));
            manager.set('tracking.useAdaptiveBirth', false);
            manager.set('tracking.maxCardinality', obj.getConfigValue(config, 'maxCardinality', 20));
            manager.set('tracking.pD', obj.getConfigValue(config, 'detectionProbability', 0.99));
            manager.set('tracking.pS', obj.getConfigValue(config, 'survivalProbability', 0.99));
            manager.set('measurement.measurementRate', obj.getConfigValue(config, 'measurementRate', 10));

            obj.Config = config;
            obj.Engine = tracking.extended.internal.GgiwEngine(manager);
            obj.State = struct('time', 0, 'estimate', obj.emptyEstimate());
            obj.Initialized = true;
        end

        function obj = predict(obj, stepContext)
            if nargin < 2 || isempty(stepContext)
                stepContext = struct();
            end
            obj = obj.ensureInitialized();
            model = obj.defaultModel();
            F = obj.getConfigValue(stepContext, 'F', model.F);
            Q = obj.getConfigValue(stepContext, 'Q', model.Q);
            obj.Engine.predict(F, Q);
            obj.State.time = obj.State.time + 1;
        end

        function obj = update(obj, measurements, stepContext)
            if nargin < 3 || isempty(stepContext)
                stepContext = struct();
            end
            obj = obj.ensureInitialized();
            model = obj.defaultModel();
            H = obj.getConfigValue(stepContext, 'H', model.H);
            R = obj.getConfigValue(stepContext, 'R', model.R);
            normalized = obj.normalizeMeasurements(measurements);
            obj.Engine.update(normalized, H, R);
            obj.Engine.pruneAndMerge(1e-10, 4, 20);
            obj.State.estimate = obj.standardizeEstimate(obj.Engine.extractStates(0.1));
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
                z = [k, k + 0.25, k + 0.5; 0.5 * k, 0.5 * k + 0.2, 0.5 * k - 0.2];
                runner = runner.predict(struct('time', k));
                runner = runner.update(z, struct('time', k));
                measurements{k} = z;
                estimates{k} = runner.estimate();
                truth{k} = struct('states', [k; 0; 0.5 * k; 0], 'extents', eye(2));
            end

            metadata = struct('algorithm', obj.getConfigValue(config, 'algorithm', 'GGIW-PHD'), ...
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

        function model = defaultModel(~)
            model = struct();
            model.F = [1, 1, 0, 0; 0, 1, 0, 0; 0, 0, 1, 1; 0, 0, 0, 1];
            model.Q = diag([0.1, 0.1, 0.1, 0.1]);
            model.H = [1, 0, 0, 0; 0, 0, 1, 0];
            model.R = eye(2) * 10;
        end

        function measurements = normalizeMeasurements(~, measurements)
            if nargin < 2 || isempty(measurements)
                measurements = {};
            elseif isnumeric(measurements)
                measurements = {struct('points', measurements)};
            elseif isstruct(measurements)
                measurements = num2cell(measurements);
            end
        end

        function estimate = standardizeEstimate(obj, raw)
            if isfield(raw, 'positions') && ~isempty(raw.positions)
                states = raw.positions;
            else
                states = zeros(4, 0);
            end
            if size(states, 1) == 2
                states = [states(1, :); zeros(1, size(states, 2)); states(2, :); zeros(1, size(states, 2))];
            end
            estimate = obj.emptyEstimate(size(states, 2));
            estimate.states = states;
            if isfield(raw, 'measurementRates') && ~isempty(raw.measurementRates)
                estimate.scores = raw.measurementRates(:)';
            else
                estimate.scores = ones(1, estimate.cardinality);
            end
            for iTarget = 1:estimate.cardinality
                estimate.labels{iTarget} = sprintf('ggiw-%d', iTarget);
                if isfield(raw, 'extents') && size(raw.extents, 3) >= iTarget
                    estimate.extents(:, :, iTarget) = obj.makePsd(raw.extents(:, :, iTarget));
                end
            end
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
