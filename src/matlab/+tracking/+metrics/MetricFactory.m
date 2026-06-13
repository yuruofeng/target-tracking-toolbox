classdef MetricFactory
    % METRICFACTORY 度量工厂类
    %
    % 使用工厂模式创建不同类型的度量对象
    %
    % 使用示例:
    %   factory = tracking.metrics.MetricFactory();
    %   gospa = factory.createMetric('GOSPA', 'p', 2, 'c', 10);
    %   [distance, assignment, decomposition] = gospa.compute(x, y);
    %

    properties (Constant)
        % 支持的度量类型
        MetricTypes = struct(...
            'GOSPA', 'GOSPA', ...
            'TRAJECTORY', 'TrajectoryMetric' ...
        )
    end
    
    methods (Static)
        function metricObj = createMetric(metricType, varargin)
            % CREATEMETRIC 创建度量对象
            %
            % 输入:
            %   metricType - 度量类型字符串
            %   varargin - 度量特定参数
            %
            % 输出:
            %   metricObj - 度量对象
            
            % 验证输入
            if nargin < 1
                ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                    '必须提供metricType参数');
                throw(ME);
            end
            
            if ~ischar(metricType) && ~isstring(metricType)
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_TYPE, ...
                    'metricType必须是字符串');
                throw(ME);
            end
            
            % 根据类型创建度量
            switch upper(metricType)
                case {'GOSPA', 'GENERALIZED_OPTIMAL_SUB_PATTERN_ASSIGNMENT'}
                    metricObj = tracking.metrics.GOSPA(varargin{:});
                    
                case {'TRAJECTORY', 'TRAJECTORY_METRIC'}
                    metricObj = tracking.metrics.TrajectoryMetric(varargin{:});
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '不支持的度量类型: %s', metricType);
                    throw(ME);
            end
        end
        
        function types = getAvailableTypes()
            % GETAVAILABLETYPES 获取可用的度量类型
            %
            % 输出:
            %   types - 可用度量类型列表
            
            types = {'GOSPA', 'TrajectoryMetric'};
        end
        
        function displayAvailableTypes()
            % DISPLAYAVAILABLETYPES 显示可用的度量类型
            
            fprintf('\n===== 可用的度量类型 =====\n');
            types = tracking.metrics.MetricFactory.getAvailableTypes();
            for i = 1:length(types)
                fprintf('%d. %s\n', i, types{i});
            end
            fprintf('=========================\n\n');
        end
        
        function [result, varargout] = compute(metricType, varargin)
            % COMPUTE 直接计算度量
            %
            % 输入:
            %   metricType - 度量类型
            %   varargin - 计算参数
            %
            % 输出:
            %   result - 主要计算结果
            %   varargout - 附加输出
            
            metricObj = tracking.metrics.MetricFactory.createMetric(metricType);
            [result, varargout{1:nargout-1}] = metricObj.compute(varargin{:});
        end
        
        function metricObj = createFromPreset(presetName, varargin)
            % CREATEFROMPRESET 从预设创建度量
            %
            % 输入:
            %   presetName - 预设名称
            %   varargin   - 额外参数
            %
            % 输出:
            %   metricObj - 度量对象
            
            switch lower(presetName)
                case 'standard'
                    % 标准GOSPA配置
                    metricObj = tracking.metrics.MetricFactory.createMetric('GOSPA', ...
                        'p', 2, 'c', 10, 'alpha', 2);
                    
                case 'trajectory'
                    % 轨迹度量配置
                    metricObj = tracking.metrics.MetricFactory.createMetric('TrajectoryMetric', ...
                        'p', 2, 'c', 10, 'alpha', 2);
                    
                case 'strict'
                    % 严格的GOSPA配置（更小的截断距离）
                    metricObj = tracking.metrics.MetricFactory.createMetric('GOSPA', ...
                        'p', 2, 'c', 5, 'alpha', 2);
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '未知的预设名称: %s', presetName);
                    throw(ME);
            end
        end
    end
end
