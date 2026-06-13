classdef TPMBFactory
    % TPMBFACTORY 轨迹PMB滤波器工厂类
    %
    % 使用工厂模式创建不同类型的轨迹PMB滤波器
    %
    % 支持的滤波器类型:
    %   'TPMBM' - 轨迹泊松多伯努利混合滤波器
    %   'TPMB'  - 轨迹泊松多伯努利滤波器
    %   'TMBM'  - 轨迹多伯努利混合滤波器
    %
    % 使用示例:
    %   factory = tracking.multi.rfs.trajectory.pmbm.TPMBFactory();
    %   filter = factory.createFilter('TPMBM', config);
    %   result = filter.run(measurements);
    %

    properties (Constant)
        FilterTypes = struct(...
            'TPMBM', 'TPMBM', ...
            'TPMB', 'TPMB', ...
            'TMBM', 'TMBM' ...
        )
    end
    
    methods (Static)
        function filter = createFilter(filterType, config)
            % CREATEFILTER 创建轨迹滤波器
            
            if nargin < 2
                ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                    '必须提供filterType和config参数');
                throw(ME);
            end
            
            switch upper(filterType)
                case 'TPMBM'
                    filter = tracking.multi.rfs.trajectory.pmbm.TPMBM(config);
                    
                case 'TPMB'
                    filter = tracking.multi.rfs.trajectory.pmb.TPMB(config);
                    
                case 'TMBM'
                    filter = tracking.multi.rfs.trajectory.mbm.TMBM(config);
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '不支持的滤波器类型: %s', filterType);
                    throw(ME);
            end
        end
        
        function types = getAvailableTypes()
            % GETAVAILABLETYPES 获取可用的滤波器类型
            
            types = {'TPMBM', 'TPMB', 'TMBM'};
        end
        
        function displayAvailableTypes()
            % DISPLAYAVAILABLETYPES 显示可用的滤波器类型
            
            fprintf('\n===== 可用的轨迹PMB滤波器类型 =====\n');
            types = tracking.multi.rfs.trajectory.pmbm.TPMBFactory.getAvailableTypes();
            for i = 1:length(types)
                fprintf('%d. %s\n', i, types{i});
            end
            fprintf('=====================================\n\n');
        end
        
        function info = getTypeInfo(filterType)
            % GETTYPEINFO 获取滤波器类型信息
            
            info = struct();
            
            switch upper(filterType)
                case 'TPMBM'
                    info.name = 'Trajectory PMBM Filter';
                    info.description = '轨迹泊松多伯努利混合滤波器';
                    info.reference = 'Garcia-Fernandez et al., IEEE TSP 2020';
                    info.features = {'完整轨迹估计', '多假设管理', '泊松新生'};
                    
                case 'TPMB'
                    info.name = 'Trajectory PMB Filter';
                    info.description = '轨迹泊松多伯努利滤波器';
                    info.reference = 'Williams, IEEE TAES 2015';
                    info.features = {'单假设估计', '投影步骤', '泊松新生'};
                    
                case 'TMBM'
                    info.name = 'Trajectory MBM Filter';
                    info.description = '轨迹多伯努利混合滤波器';
                    info.reference = 'Garcia-Fernandez et al., IEEE TSP 2020';
                    info.features = {'完整轨迹估计', '多假设管理', '无泊松分量'};
                    
                otherwise
                    info.name = 'Unknown';
                    info.description = '';
                    info.reference = '';
                    info.features = {};
            end
        end
        
        function filter = createFromPreset(presetName, varargin)
            % CREATEFROMPRESET 从预设配置创建滤波器
            
            config = tracking.multi.rfs.trajectory.pmbm.TPMBFactory.getPresetConfig(presetName, varargin{:});
            filterType = tracking.multi.rfs.trajectory.pmbm.TPMBFactory.getPresetFilterType(presetName);
            filter = tracking.multi.rfs.trajectory.pmbm.TPMBFactory.createFilter(filterType, config);
        end
        
        function config = getPresetConfig(presetName, varargin)
            % GETPRESETCONFIG 获取预设配置
            
            config = tracking.multi.rfs.core.FilterConfig();
            
            switch lower(presetName)
                case 'default'
                    config.detectionProb = 0.9;
                    config.survivalProb = 0.99;
                    config.pruningThreshold = 1e-4;
                    config.maxComponents = 100;
                    config.existenceThreshold = 1e-5;
                    
                case 'trajectory_focused'
                    config.detectionProb = 0.9;
                    config.survivalProb = 0.99;
                    config.pruningThreshold = 1e-5;
                    config.maxComponents = 150;
                    config.existenceThreshold = 1e-6;
                    config.trajectoryLength = 10;
                    
                case 'fast'
                    config.detectionProb = 0.85;
                    config.survivalProb = 0.95;
                    config.pruningThreshold = 1e-3;
                    config.maxComponents = 50;
                    config.existenceThreshold = 1e-4;
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '未知的预设名称: %s', presetName);
                    throw(ME);
            end
            
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                if isprop(config, key)
                    config.(key) = value;
                end
            end
        end
        
        function filterType = getPresetFilterType(presetName)
            % GETPRESETFILTERTYPE 获取预设对应的滤波器类型
            
            filterType = 'TPMBM';
        end
    end
end
