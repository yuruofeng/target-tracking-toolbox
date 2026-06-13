classdef (Abstract) BaseFilter
    % BASEFILTER 滤波器抽象基类
    %
    % 定义所有滤波器的标准接口和通用流程
    % 子类必须实现以下抽象方法:
    %   - initialize()
    %   - predict()
    %   - update()
    %   - estimate()
    %   - prune()
    %
    % 使用示例:
    %   classdef GMPHD < BaseFilter
    %       methods
    %           function obj = initialize(obj)
    %               % 实现初始化
    %           end
    %       end
    %   end
    %

    properties (Access = protected)
        Config           % FilterConfig对象
        State            % 滤波器状态
        CurrentTime = 0  % 当前时刻
    end
    
    properties (Access = public)
        History          % 历史记录
    end
    
    methods
        function obj = BaseFilter(config)
            % BASEFILTER 构造函数
            %
            % 输入:
            %   config - FilterConfig对象
            
            if nargin < 1
                ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                    '必须提供config参数');
                throw(ME);
            end
            
            % 验证配置
            if ~isa(config, 'tracking.multi.rfs.core.FilterConfig')
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_TYPE, ...
                    'config必须是FilterConfig对象，实际为 %s', class(config));
                throw(ME);
            end
            
            obj.Config = config;
            obj.CurrentTime = 0;
            obj.History = struct();
        end
        
        function result = run(obj, measurements, groundTruth)
            % RUN 执行滤波器（模板方法）
            %
            % 定义滤波器的标准执行流程
            %
            % 输入:
            %   measurements - 量测数据结构
            %   groundTruth  - (可选) 真实数据，用于性能评估
            %
            % 输出:
            %   result - FilterResult对象

            % 处理可选参数
            if nargin < 3
                groundTruth = [];
            end

            % 初始化结果对象
            result = tracking.multi.rfs.core.FilterResult();
            result.algorithmName = class(obj);

            % 记录开始时间
            startTime = tic;

            try
                % 1. 验证输入
                obj = obj.validateInputs(measurements, groundTruth);
                
                % 2. 初始化滤波器
                obj = obj.initialize();
                
                % 3. 主滤波循环
                numSteps = length(measurements.timeStamps);
                estimates = cell(1, numSteps);
                
                for k = 1:numSteps
                    obj.CurrentTime = k;
                    
                    % 3.1 预测
                    obj = obj.predict();
                    
                    % 3.2 更新
                    obj = obj.update(measurements.measurements{k});
                    
                    % 3.3 估计
                    estimates{k} = obj.estimate();
                    
                    % 3.4 剪枝
                    obj = obj.prune();
                    
                    % 3.5 记录历史（可选）
                    if obj.Config.verbose
                        obj = obj.recordHistory(k);
                    end
                end
                
                % 4. 组织结果
                organized = obj.organizeEstimates(estimates);
                result.estimates.estimatesList = organized.estimatesList;
                result.estimates.weights = organized.weights;
                result.estimates.cardinality = organized.cardinality;
                result.estimates.trajectories = organized.trajectories;
                result.filterState = obj.State;
                
                % 5. 计算性能度量（如果有真实数据）
                if ~isempty(groundTruth)
                    result.metrics = obj.computeMetrics(result.estimates, groundTruth);
                end
                
                result.status = 'success';
                
            catch ME
                result.status = 'error';
                % 尝试将异常标识符映射到错误码
                try
                    % 检查是否是MTTException
                    if isa(ME, 'tracking.core.MTTException')
                        result.errorCode = ME.ErrorCode;
                    else
                        % 对于其他异常，使用通用错误码
                        result.errorCode = tracking.core.ErrorCode.GENERAL_ERROR;
                    end
                catch
                    % 如果映射失败，使用通用错误码
                    result.errorCode = tracking.core.ErrorCode.GENERAL_ERROR;
                end
                result.message = ME.message;

                % 记录错误堆栈到诊断信息
                result.diagnostics.errorStack = ME.stack;
                result.diagnostics.errorIdentifier = ME.identifier;

                if obj.Config.verbose
                    fprintf('滤波器执行失败: %s\n', ME.message);
                    fprintf('错误标识符: %s\n', ME.identifier);
                    fprintf('错误堆栈:\n');
                    for stack_i = 1:length(ME.stack)
                        fprintf('  [%d] %s (文件: %s, 行: %d)\n', ...
                            stack_i, ME.stack(stack_i).name, ME.stack(stack_i).file, ME.stack(stack_i).line);
                    end
                end
            end
            
            % 记录执行时间
            result.executionTime = toc(startTime);
            
            % 显示结果摘要
            if obj.Config.verbose
                result.display();
            end
        end
    end
    
    % 抽象方法（子类必须实现）
    methods (Abstract)
        obj = initialize(obj)
        obj = predict(obj)
        obj = update(obj, measurement)
        estimate = estimate(obj)
        obj = prune(obj)
    end
    
    % 可选方法（子类可覆盖）
    methods (Access = protected)
        function obj = validateInputs(obj, measurements, groundTruth)
            % VALIDATEINPUTS 验证输入数据
            
            % 验证量测数据
            if ~isstruct(measurements)
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_INPUT, ...
                    'measurements必须是结构体');
                throw(ME);
            end

            if ~isfield(measurements, 'timeStamps') || ...
               ~isfield(measurements, 'measurements')
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_INPUT, ...
                    'measurements必须包含timeStamps和measurements字段');
                throw(ME);
            end

            % 验证真实数据（如果提供）
            if nargin > 2 && ~isempty(groundTruth)
                if ~isstruct(groundTruth)
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_INPUT, ...
                        'groundTruth必须是结构体');
                    throw(ME);
                end
            end
        end
        
        function obj = recordHistory(obj, k)
            % RECORDHISTORY 记录历史状态
            
            if ~isfield(obj.History, 'states')
                obj.History.states = {};
            end
            
            obj.History.states{k} = obj.State;
        end
        
        function estimates = organizeEstimates(obj, estimates)
            % ORGANIZEESTIMATES 组织估计结果

            % Extract states and cardinalities from estimates cell array
            numSteps = numel(estimates);
            cardinalities = zeros(1, numSteps);

            for k = 1:numSteps
                if isfield(estimates{k}, 'cardinality')
                    cardinalities(k) = estimates{k}.cardinality;
                end
            end

            % Create struct with all required fields
            result_struct = struct();
            result_struct.estimatesList = estimates;
            result_struct.weights = [];  % Can be populated by specific filters if needed
            result_struct.cardinality = cardinalities;
            result_struct.trajectories = {};  % Can be populated by trajectory filters

            estimates = result_struct;
        end
        
        function metrics = computeMetrics(obj, estimates, groundTruth)
            % COMPUTEMETRICS 计算性能度量
            
            metrics = struct();
            
            % 这里可以添加默认的度量计算
            % 子类可以覆盖此方法添加特定度量
        end
    end
    
    methods (Access = public)
        function state = getState(obj)
            % GETSTATE 获取当前滤波器状态
            
            state = obj.State;
        end
        
        function obj = setState(obj, state)
            % SETSTATE 设置滤波器状态
            
            obj.State = state;
        end
        
        function config = getConfig(obj)
            % GETCONFIG 获取配置
            
            config = obj.Config;
        end
    end
end
