classdef PythonModelAdapter < handle
% PYTHONMODELADAPTER  MATLAB adapter for Python deep-learning trackers.

    properties
        ModuleName
        FunctionName
    end

    methods
        function obj = PythonModelAdapter(moduleName, functionName)
            if nargin < 1 || isempty(moduleName)
                moduleName = 'target_tracking_dl';
            end
            if nargin < 2 || isempty(functionName)
                functionName = 'predict';
            end
            obj.ModuleName = moduleName;
            obj.FunctionName = functionName;
        end

        function result = predict(obj, sequence, config)
        % PREDICT  Call the configured Python prediction function.
            if nargin < 3
                config = struct();
            end

            module = py.importlib.import_module(obj.ModuleName);
            pyFunc = module.(obj.FunctionName);
            pyResult = pyFunc(sequence, config);
            result = tracking.learning.PythonModelAdapter.toMatlab(pyResult);
        end
    end

    methods (Static)
        function value = toMatlab(pyValue)
            if isa(pyValue, 'py.dict')
                keys = cell(pyValue.keys());
                value = struct();
                for iKey = 1:numel(keys)
                    key = char(keys{iKey});
                    value.(matlab.lang.makeValidName(key)) = ...
                        tracking.learning.PythonModelAdapter.toMatlab(pyValue{keys{iKey}});
                end
            elseif isa(pyValue, 'py.list') || isa(pyValue, 'py.tuple')
                items = cell(pyValue);
                value = cellfun(@tracking.learning.PythonModelAdapter.toMatlab, ...
                    items, 'UniformOutput', false);
            else
                value = pyValue;
            end
        end
    end
end
