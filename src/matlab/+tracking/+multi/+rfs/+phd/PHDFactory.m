classdef PHDFactory
    % PHDFACTORY PHD滤波器工厂类
    %
    % 使用工厂模式创建不同类型的PHD滤波器
    %
    % 使用示例:
    %   factory = tracking.multi.rfs.phd.PHDFactory();
    %   phdFilter = factory.createFilter('GM-PHD', config);
    %   result = phdFilter.run(measurements);
    %

    properties (Constant)
        % 支持的滤波器类型
        FilterTypes = struct(...
            'GM_PHD', 'GMPHD', ...
            'GM_CPHD', 'GMCPHD', ...
            'GM_TPHD', 'GMTPHD', ...
            'GM_CD_PHD', 'CDGMPHD' ...
        )
    end
    
    methods (Static)
        function filter = createFilter(filterType, config)
            % CREATEFILTER 创建PHD滤波器
            %
            % 输入:
            %   filterType - 滤波器类型字符串
            %   config     - FilterConfig对象
            %
            % 输出:
            %   filter - 滤波器对象
            
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
            
            % 根据类型创建滤波器
            switch upper(strrep(filterType, '-', '_'))
                case {'GM_PHD', 'GMPHD', 'PHD'}
                    filter = tracking.multi.rfs.phd.GMPHD(config);
                    
                case {'GM_CPHD', 'GMCPHD', 'CPHD'}
                    filter = tracking.multi.rfs.phd.GMCPHD(config);
                    
                case {'GM_TPHD', 'GMTPHD', 'TPHD'}
                    filter = tracking.multi.rfs.trajectory.phd.GMTPHD(config);
                    
                case {'GM_CD_PHD', 'CDGMPHD', 'CD_PHD', 'CDPHD'}
                    filter = tracking.multi.rfs.cd.CDGMPHD(config);
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '不支持的滤波器类型: %s', filterType);
                    throw(ME);
            end
        end
        
        function types = getAvailableTypes()
            % GETAVAILABLETYPES 获取可用的滤波器类型
            %
            % 输出:
            %   types - 可用滤波器类型列表
            
            types = {'GM-PHD', 'GM-CPHD', 'GM-TPHD', 'GM-CD-PHD'};
        end
        
        function displayAvailableTypes()
            % DISPLAYAVAILABLETYPES 显示可用的滤波器类型
            
            fprintf('\n===== 可用的PHD滤波器类型 =====\n');
            types = tracking.multi.rfs.phd.PHDFactory.getAvailableTypes();
            for i = 1:length(types)
                fprintf('%d. %s\n', i, types{i});
            end
            fprintf('==============================\n\n');
        end
        
        function filter = createFromPreset(presetName, varargin)
            % CREATEFROMPRESET 从预设配置创建滤波器
            %
            % 输入:
            %   presetName - 预设名称
            %   varargin   - 额外配置参数
            %
            % 输出:
            %   filter - 滤波器对象
            
            % 获取预设配置
            config = tracking.multi.rfs.phd.PHDFactory.getPresetConfig(presetName, varargin{:});
            
            % 创建滤波器
            filterType = tracking.multi.rfs.phd.PHDFactory.getPresetFilterType(presetName);
            filter = tracking.multi.rfs.phd.PHDFactory.createFilter(filterType, config);
        end
        
        function config = getPresetConfig(presetName, varargin)
            % GETPRESETCONFIG 获取预设配置
            %
            % 输入:
            %   presetName - 预设名称
            %   varargin   - 额外配置参数
            %
            % 输出:
            %   config - FilterConfig对象
            
            % 创建默认配置
            config = tracking.multi.rfs.core.FilterConfig();
            
            % 应用预设
            switch lower(presetName)
                case 'default'
                    % 默认配置
                    config.detectionProb = 0.9;
                    config.survivalProb = 0.99;
                    config.pruningThreshold = 1e-5;
                    config.mergingThreshold = 0.1;
                    config.maxComponents = 100;
                    
                case 'high_precision'
                    % 高精度配置
                    config.detectionProb = 0.95;
                    config.survivalProb = 0.99;
                    config.pruningThreshold = 1e-6;
                    config.mergingThreshold = 0.05;
                    config.maxComponents = 200;
                    
                case 'fast'
                    % 快速配置
                    config.detectionProb = 0.85;
                    config.survivalProb = 0.95;
                    config.pruningThreshold = 1e-4;
                    config.mergingThreshold = 0.2;
                    config.maxComponents = 50;
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '未知的预设名称: %s', presetName);
                    throw(ME);
            end
            
            % 应用额外参数
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
            
            switch lower(presetName)
                case {'default', 'high_precision', 'fast'}
                    filterType = 'GM-PHD';
                otherwise
                    filterType = 'GM-PHD';
            end
        end
    end
end
