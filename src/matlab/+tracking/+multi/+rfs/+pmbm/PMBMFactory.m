classdef PMBMFactory
    % PMBMFACTORY PMBM/PMB滤波器工厂类
    %
    % 使用工厂模式创建不同类型的PMBM和PMB滤波器
    %
    % 支持的滤波器类型:
    %   'PMBM'  - 泊松多伯努利混合滤波器
    %   'PMB'   - 泊松多伯努利滤波器
    %   'TPMBM' - 轨迹泊松多伯努利混合滤波器
    %
    % 使用示例:
    %   factory = tracking.multi.rfs.pmbm.PMBMFactory();
    %   pmbmFilter = factory.createFilter('PMBM', config);
    %   result = pmbmFilter.run(measurements);
    %

    properties (Constant)
        FilterTypes = struct(...
            'PMBM', 'PMBM', ...
            'PMB', 'PMB', ...
            'TPMBM', 'TPMBM' ...
        )
    end
    
    methods (Static)
        function filter = createFilter(filterType, config)
            % CREATEFILTER 创建滤波器
            
            if nargin < 2
                ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                    '必须提供filterType和config参数');
                throw(ME);
            end
            
            switch upper(filterType)
                case 'PMBM'
                    filter = tracking.multi.rfs.pmbm.PMBM(config);
                    
                case 'PMB'
                    filter = tracking.multi.rfs.pmbm.PMB(config);
                    
                case 'TPMBM'
                    filter = tracking.multi.rfs.trajectory.pmbm.TPMBM(config);
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '不支持的滤波器类型: %s', filterType);
                    throw(ME);
            end
        end
        
        function types = getAvailableTypes()
            % GETAVAILABLETYPES 获取可用的滤波器类型
            
            types = {'PMBM', 'PMB', 'TPMBM'};
        end
        
        function displayAvailableTypes()
            % DISPLAYAVAILABLETYPES 显示可用的滤波器类型
            
            fprintf('\n===== 可用的PMBM滤波器类型 =====\n');
            types = tracking.multi.rfs.pmbm.PMBMFactory.getAvailableTypes();
            for i = 1:length(types)
                fprintf('%d. %s\n', i, types{i});
            end
            fprintf('================================\n\n');
        end
        
        function filter = createFromPreset(presetName, varargin)
            % CREATEFROMPRESET 从预设配置创建滤波器
            
            config = tracking.multi.rfs.pmbm.PMBMFactory.getPresetConfig(presetName, varargin{:});
            filterType = tracking.multi.rfs.pmbm.PMBMFactory.getPresetFilterType(presetName);
            filter = tracking.multi.rfs.pmbm.PMBMFactory.createFilter(filterType, config);
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
                    
                case 'high_precision'
                    config.detectionProb = 0.95;
                    config.survivalProb = 0.999;
                    config.pruningThreshold = 1e-5;
                    config.maxComponents = 200;
                    config.existenceThreshold = 1e-6;
                    
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
            
            filterType = 'PMBM';
        end
    end
end
