classdef TPHDFactory
    % TPHDFACTORY TPHD滤波器工厂类
    %
    % 使用工厂模式创建不同类型的TPHD滤波器
    %
    % 使用示例:
    %   factory = tracking.multi.rfs.trajectory.phd.TPHDFactory();
    %   filter = factory.createFilter('GMTPHD', config);
    %

    properties (Constant)
        % 支持的滤波器类型
        FilterTypes = struct(...
            'GMTPHD', 'GMTPHD' ...
        )
    end
    
    methods (Static)
        function filter = createFilter(filterType, config)
            % CREATEFILTER 创建TPHD滤波器
            %
            % 输入:
            %   filterType - 滤波器类型字符串
            %   config - FilterConfig对象
            %
            % 输出:
            %   filter - TPHD滤波器对象
            
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
            if ~isfield(config.extraParams, 'Lscan')
                config.extraParams.Lscan = 5;  % 默认值
            end
            
            if ~isfield(config.extraParams, 'maxComponents')
                config.extraParams.maxComponents = 30;  % 默认值
            end
            
            if ~isfield(config.extraParams, 'absorptionThreshold')
                config.extraParams.absorptionThreshold = 4;  % 默认值
            end
            
            % 根据类型创建滤波器
            switch upper(filterType)
                case 'GMTPHD'
                    filter = tracking.multi.rfs.trajectory.phd.GMTPHD(config);
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '不支持的TPHD滤波器类型: %s', filterType);
                    throw(ME);
            end
        end
        
        function types = getAvailableTypes()
            % GETAVAILABLETYPES 获取可用的滤波器类型
            %
            % 输出:
            %   types - 可用滤波器类型列表
            
            types = {'GMTPHD'};
        end
        
        function displayAvailableTypes()
            % DISPLAYAVAILABLETYPES 显示可用的滤波器类型
            
            fprintf('\n===== 可用的TPHD滤波器类型 =====\n');
            types = tracking.multi.rfs.trajectory.phd.TPHDFactory.getAvailableTypes();
            for i = 1:length(types)
                fprintf('%d. %s\n', i, types{i});
            end
            fprintf('=============================\n\n');
        end
        
        function filter = createFromPreset(presetName, varargin)
            % CREATEFROMPRESET 从预设创建TPHD滤波器
            %
            % 输入:
            %   presetName - 预设名称
            %   varargin - 额外参数
            %
            % 输出:
            %   filter - TPHD滤波器对象
            
            % 创建默认配置
            config = tracking.multi.rfs.core.FilterConfig();
            
            % 根据预设设置参数
            switch lower(presetName)
                case 'standard'
                    % 标准GMTPHD配置
                    config.extraParams.Lscan = 5;
                    config.extraParams.maxComponents = 30;
                    config.extraParams.absorptionThreshold = 4;
                    config.pruningThreshold = 1e-4;
                    
                case 'fast'
                    % 快速GMTPHD配置（较少的组件）
                    config.extraParams.Lscan = 3;
                    config.extraParams.maxComponents = 15;
                    config.extraParams.absorptionThreshold = 5;
                    config.pruningThreshold = 1e-3;
                    
                case 'accurate'
                    % 精确GMTPHD配置（更多的组件）
                    config.extraParams.Lscan = 7;
                    config.extraParams.maxComponents = 50;
                    config.extraParams.absorptionThreshold = 3;
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
                        case 'lscan'
                            config.extraParams.Lscan = value;
                        case 'maxcomponents'
                            config.extraParams.maxComponents = value;
                        case 'absorptionthreshold'
                            config.extraParams.absorptionThreshold = value;
                    end
                end
            end
            
            % 创建滤波器
            filter = tracking.multi.rfs.trajectory.phd.TPHDFactory.createFilter('GMTPHD', config);
        end
    end
end
