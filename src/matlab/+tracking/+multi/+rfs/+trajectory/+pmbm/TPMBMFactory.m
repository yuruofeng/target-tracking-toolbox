classdef TPMBMFactory
    % TPMBMFACTORY TPMBM滤波器工厂类
    %
    % 使用工厂模式创建不同类型的TPMBM滤波器
    %
    % 使用示例:
    %   factory = tracking.multi.rfs.trajectory.pmbm.TPMBMFactory();
    %   filter = factory.createFilter('TPMBM', config);
    %

    properties (Constant)
        % 支持的滤波器类型
        FilterTypes = struct(...
            'TPMBM', 'TPMBM' ...
        )
    end
    
    methods (Static)
        function filter = createFilter(filterType, config)
            % CREATEFILTER 创建TPMBM滤波器
            %
            % 输入:
            %   filterType - 滤波器类型字符串
            %   config - FilterConfig对象
            %
            % 输出:
            %   filter - TPMBM滤波器对象
            
            % 验证输入
            if nargin < 2
                ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                    '必须提供filterType和config参数');
                throw(ME);
            end
            
            if ~ischar(filterType) && ~isstring(filterType)
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_TYPE, ...
                    'filterType必须是字符串');
                throw(ME);
            end
            
            if ~isa(config, 'tracking.multi.rfs.core.FilterConfig')
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_TYPE, ...
                    'config必须是tracking.multi.rfs.core.FilterConfig对象');
                throw(ME);
            end
            
            % 检查必要的额外参数
            if ~isfield(config.extraParams, 'gateSize')
                config.extraParams.gateSize = 9.210;  % 默认值 (99.9% 百分位数)
            end
            
            if ~isfield(config.extraParams, 'maxGlobalHypotheses')
                config.extraParams.maxGlobalHypotheses = 1000;  % 默认值
            end
            
            if ~isfield(config.extraParams, 'minGlobalHypothesisWeight')
                config.extraParams.minGlobalHypothesisWeight = 1e-4;  % 默认值
            end
            
            if ~isfield(config.extraParams, 'numMinimumAssignment')
                config.extraParams.numMinimumAssignment = 100;  % 默认值
            end
            
            if ~isfield(config.extraParams, 'minEndTimeProbability')
                config.extraParams.minEndTimeProbability = 1e-4;  % 默认值
            end
            
            if ~isfield(config.extraParams, 'minBirthTimeProbability')
                config.extraParams.minBirthTimeProbability = 1e-1;  % 默认值
            end
            
            if ~isfield(config.extraParams, 'minExistenceProbability')
                config.extraParams.minExistenceProbability = 1e-4;  % 默认值
            end
            
            if ~isfield(config.extraParams, 'totalTimeSteps')
                config.extraParams.totalTimeSteps = 100;  % 默认值
            end
            
            % 根据类型创建滤波器
            switch upper(filterType)
                case 'TPMBM'
                    filter = tracking.multi.rfs.trajectory.pmbm.TPMBM(config);
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '不支持的TPMBM滤波器类型: %s', filterType);
                    throw(ME);
            end
        end
        
        function types = getAvailableTypes()
            % GETAVAILABLETYPES 获取可用的滤波器类型
            %
            % 输出:
            %   types - 可用滤波器类型列表
            
            types = {'TPMBM'};
        end
        
        function displayAvailableTypes()
            % DISPLAYAVAILABLETYPES 显示可用的滤波器类型
            
            fprintf('\n===== 可用的TPMBM滤波器类型 =====\n');
            types = tracking.multi.rfs.trajectory.pmbm.TPMBMFactory.getAvailableTypes();
            for i = 1:length(types)
                fprintf('%d. %s\n', i, types{i});
            end
            fprintf('=============================\n\n');
        end
        
        function filter = createFromPreset(presetName, varargin)
            % CREATEFROMPRESET 从预设创建TPMBM滤波器
            %
            % 输入:
            %   presetName - 预设名称
            %   varargin - 额外参数
            %
            % 输出:
            %   filter - TPMBM滤波器对象
            
            % 创建默认配置
            config = tracking.multi.rfs.core.FilterConfig();
            
            % 根据预设设置参数
            switch lower(presetName)
                case 'standard'
                    % 标准TPMBM配置
                    config.extraParams.gateSize = 9.210;
                    config.extraParams.maxGlobalHypotheses = 1000;
                    config.extraParams.minGlobalHypothesisWeight = 1e-4;
                    config.extraParams.numMinimumAssignment = 100;
                    config.extraParams.minEndTimeProbability = 1e-4;
                    config.extraParams.minBirthTimeProbability = 1e-1;
                    config.extraParams.minExistenceProbability = 1e-4;
                    config.pruningThreshold = 1e-4;
                    
                case 'fast'
                    % 快速TPMBM配置（较少的全局假设）
                    config.extraParams.gateSize = 9.210;
                    config.extraParams.maxGlobalHypotheses = 500;
                    config.extraParams.minGlobalHypothesisWeight = 1e-3;
                    config.extraParams.numMinimumAssignment = 50;
                    config.extraParams.minEndTimeProbability = 1e-3;
                    config.extraParams.minBirthTimeProbability = 1e-1;
                    config.extraParams.minExistenceProbability = 1e-3;
                    config.pruningThreshold = 1e-3;
                    
                case 'accurate'
                    % 精确TPMBM配置（更多的全局假设）
                    config.extraParams.gateSize = 9.210;
                    config.extraParams.maxGlobalHypotheses = 2000;
                    config.extraParams.minGlobalHypothesisWeight = 1e-5;
                    config.extraParams.numMinimumAssignment = 200;
                    config.extraParams.minEndTimeProbability = 1e-5;
                    config.extraParams.minBirthTimeProbability = 1e-2;
                    config.extraParams.minExistenceProbability = 1e-5;
                    config.pruningThreshold = 1e-5;
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '未知的预设名称: %s', presetName);
                    throw(ME);
            end
            
            % 解析额外参数
            for i = 1:2:length(varargin)
                if ischar(varargin{i}) || isstring(varargin{i})
                    key = lower(varargin{i});
                    value = varargin{i+1};
                    
                    switch key
                        case 'motionmodel'
                            config.motionModel = value;
                        case 'measurementmodel'
                            config.measurementModel = value;
                        case 'gatesize'
                            config.extraParams.gateSize = value;
                        case 'maxglobalhypotheses'
                            config.extraParams.maxGlobalHypotheses = value;
                        case 'totalTimesteps'
                            config.extraParams.totalTimeSteps = value;
                    end
                end
            end
            
            % 创建滤波器
            filter = tracking.multi.rfs.trajectory.pmbm.TPMBMFactory.createFilter('TPMBM', config);
        end
    end
end
