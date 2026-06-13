classdef CDFilterFactory
    % CDFILTERFACTORY 连续-离散滤波器工厂类
    %
    % 用于创建和管理各种连续-离散滤波器的实例
    %
    % 支持的滤波器类型:
    %   'cd_gmphd'   - 连续-离散高斯混合PHD滤波器
    %   'cd_gmcphd'  - 连续-离散高斯混合CPHD滤波器
    %   'cd_pmbm'    - 连续-离散PMBM滤波器
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   filter = tracking.multi.rfs.cd.CDFilterFactory.create('cd_gmphd', config);
    %

    methods (Static)
        function filter = create(filterType, config)
            % CREATE 创建滤波器实例
            %
            % 输入:
            %   filterType - 滤波器类型字符串
            %   config     - FilterConfig配置对象
            %
            % 输出:
            %   filter - 滤波器实例
            
            switch lower(filterType)
                case 'cd_gmphd'
                    filter = tracking.multi.rfs.cd.CDGMPHD(config);
                case 'cd_gmcphd'
                    filter = tracking.multi.rfs.cd.CDGMCPHD(config);
                case 'cd_pmbm'
                    filter = tracking.multi.rfs.cd.CDPMBM(config);
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNKNOWN_FILTER_TYPE, ...
                        '未知的连续-离散滤波器类型: %s', filterType);
                    throw(ME);
            end
        end
        
        function types = getAvailableTypes()
            % GETAVAILABLETYPES 获取可用的滤波器类型
            %
            % 输出:
            %   types - 滤波器类型元胞数组
            
            types = {'cd_gmphd', 'cd_gmcphd', 'cd_pmbm'};
        end
        
        function info = getTypeInfo(filterType)
            % GETTYPEINFO 获取滤波器类型信息
            %
            % 输入:
            %   filterType - 滤波器类型字符串
            %
            % 输出:
            %   info - 包含类型信息的结构体
            
            info = struct();
            
            switch lower(filterType)
                case 'cd_gmphd'
                    info.name = 'Continuous-Discrete GM-PHD Filter';
                    info.description = '连续-离散高斯混合PHD滤波器';
                    info.reference = 'A. F. García-Fernández, S. Maskell, "Continuous-discrete multiple target filtering"';
                case 'cd_gmcphd'
                    info.name = 'Continuous-Discrete GM-CPHD Filter';
                    info.description = '连续-离散高斯混合CPHD滤波器';
                    info.reference = 'A. F. García-Fernández, S. Maskell, "Continuous-discrete multiple target filtering"';
                case 'cd_pmbm'
                    info.name = 'Continuous-Discrete PMBM Filter';
                    info.description = '连续-离散泊松多伯努利混合滤波器';
                    info.reference = 'A. F. García-Fernández, S. Maskell, "Continuous-discrete multiple target filtering"';
                otherwise
                    info.name = 'Unknown';
                    info.description = '';
                    info.reference = '';
            end
        end
    end
end
