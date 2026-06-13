classdef TargetPmbmFilter
% TARGETPMBMFILTER  Adapter for the extended-target PMBM target filter.

    properties
        Config struct = struct()
        State struct = struct()
        Initialized (1, 1) logical = false
    end

    methods
        function obj = TargetPmbmFilter(config)
            if nargin > 0 && ~isempty(config)
                obj.Config = config;
            end
            obj.State = struct('time', 0, 'measurements', [], 'estimate', []);
        end

        function obj = initialize(obj, initialCondition)
            if nargin < 2 || isempty(initialCondition)
                initialCondition = struct();
            end
            obj.State.initialCondition = initialCondition;
            obj.State.time = 0;
            obj.State.estimate = tracking.extended.pmbm.internal.measurementToEstimate([], 'T', false);
            obj.Initialized = true;
        end

        function obj = predict(obj, stepContext)
            if nargin < 2
                stepContext = struct();
            end
            obj = obj.ensureInitialized();
            obj.State.time = obj.State.time + 1;
            obj.State.stepContext = stepContext;
        end

        function obj = update(obj, measurements, stepContext)
            if nargin < 3
                stepContext = struct();
            end
            obj = obj.ensureInitialized();
            obj.State.measurements = measurements;
            obj.State.updateContext = stepContext;
            obj.State.estimate = tracking.extended.pmbm.internal.measurementToEstimate(measurements, 'T', false);
        end

        function estimate = estimate(obj)
            if isfield(obj.State, 'estimate') && ~isempty(obj.State.estimate)
                estimate = obj.State.estimate;
            else
                estimate = tracking.extended.pmbm.internal.measurementToEstimate([], 'T', false);
            end
        end

        function result = run(obj, scenarioOrData, varargin)
            if nargin < 2 || isempty(scenarioOrData)
                scenarioOrData = obj.Config;
            end
            config = obj.mergeConfig(scenarioOrData, varargin{:});
            result = tracking.extended.pmbm.internal.runSmokeScenario( ...
                'Extended-Target-PMBM-Filter', config, 'extended-target-pmbm');
        end
    end

    methods (Access = private)
        function obj = ensureInitialized(obj)
            if ~obj.Initialized
                obj = obj.initialize();
            end
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
    end
end
