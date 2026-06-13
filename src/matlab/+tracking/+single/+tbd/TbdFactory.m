classdef TbdFactory
% TBD.TBDFACTORY  TBD算法工厂类。
%   提供统一的TBD算法创建接口，遵循工厂设计模式。
%
%   支持的TBD算法类型：
%       'DP' - 动态规划TBD (DpTbd)
%       'PF' - 粒子滤波TBD (PfTbd)
%
%   使用方法：
%       algo = tracking.single.tbd.TbdFactory.create('DP', config);
%       algo = tracking.single.tbd.TbdFactory.create('PF', config);
%       [dp, pf] = tracking.single.tbd.TbdFactory.createAll(config);
%
%   See also: tracking.single.tbd.DpTbd, tracking.single.tbd.PfTbd, tracking.single.tbd.BaseTbd

    properties (Constant)
        SUPPORTED_TYPES = {'DP', 'PF', 'DPTBD', 'PFTBD'}
    end

    methods (Static)
        function algo = create(algoType, config)
        % CREATE  创建指定类型的TBD算法。
        %
        %   输入参数：
        %       algoType - 算法类型 ('DP', 'PF', 'DpTbd', 'PfTbd')
        %       config   - (可选) tracking.single.tbd.Config配置对象
        %
        %   输出参数：
        %       algo - TBD算法对象
            if nargin < 2
                config = tracking.single.tbd.Config();
            end
            
            switch upper(algoType)
                case {'DP', 'DPTBD'}
                    algo = tracking.single.tbd.DpTbd(config);
                    
                case {'PF', 'PFTBD'}
                    algo = tracking.single.tbd.PfTbd(config);
                    
                otherwise
                    error('tbd:UnsupportedType', ...
                        '不支持的TBD算法类型: %s。支持的类型: %s', ...
                        algoType, strjoin(tracking.single.tbd.TbdFactory.SUPPORTED_TYPES, ', '));
            end
        end
        
        function [dpAlgo, pfAlgo] = createAll(config)
        % CREATEALL  创建所有TBD算法实例。
        %
        %   输入参数：
        %       config - (可选) tracking.single.tbd.Config配置对象
        %
        %   输出参数：
        %       dpAlgo - DP-TBD算法对象
        %       pfAlgo - PF-TBD算法对象
            if nargin < 1
                config = tracking.single.tbd.Config();
            end
            
            dpAlgo = tracking.single.tbd.DpTbd(config);
            pfAlgo = tracking.single.tbd.PfTbd(config);
        end
        
        function algo = createDefault(algoType)
        % CREATEDEFAULT  使用默认配置创建TBD算法。
        %
        %   输入参数：
        %       algoType - 算法类型 ('DP' 或 'PF')
        %
        %   输出参数：
        %       algo - TBD算法对象
            algo = tracking.single.tbd.TbdFactory.create(algoType);
        end
        
        function types = getSupportedTypes()
        % GETSUPPORTEDTYPES  获取支持的算法类型列表。
            types = tracking.single.tbd.TbdFactory.SUPPORTED_TYPES;
        end
        
        function displayInfo(algoType)
        % DISPLAYINFO  显示算法类型信息。
            if nargin < 1
                fprintf('支持的TBD算法类型:\n');
                for i = 1:length(tracking.single.tbd.TbdFactory.SUPPORTED_TYPES)
                    fprintf('  - %s\n', tracking.single.tbd.TbdFactory.SUPPORTED_TYPES{i});
                end
            else
                switch upper(algoType)
                    case {'DP', 'DPTBD'}
                        fprintf('DP-TBD: 动态规划检测前跟踪\n');
                        fprintf('  - 状态向量: [行, 列]\n');
                        fprintf('  - 适用场景: 弱目标检测\n');
                        fprintf('  - 计算复杂度: O(N*T*V^2)\n');
                    case {'PF', 'PFTBD'}
                        fprintf('PF-TBD: 粒子滤波检测前跟踪\n');
                        fprintf('  - 状态向量: [行, 列, vRow, vCol, 幅度]\n');
                        fprintf('  - 适用场景: 非线性非高斯系统\n');
                        fprintf('  - 计算复杂度: O(N*P*R^2)\n');
                    otherwise
                        fprintf('未知算法类型: %s\n', algoType);
                end
            end
        end
        
        function compareAlgorithms(config, trueState, measData, psfKernel)
        % COMPAREALGORITHMS  比较不同TBD算法的性能。
        %
        %   输入参数：
        %       config     - TBD配置
        %       trueState  - 真实状态
        %       measData   - 测量数据
        %       psfKernel  - PSF核
            if nargin < 4
                error('tbd:InsufficientArgs', '需要提供config, trueState, measData, psfKernel');
            end
            
            fprintf('\n=== TBD算法性能比较 ===\n');
            
            dpAlgo = tracking.single.tbd.DpTbd(config);
            tic;
            dpAlgo.run(measData, psfKernel);
            dpTime = toc;
            [dpPosRmse, ~] = dpAlgo.computeRmse(trueState);
            
            pfAlgo = tracking.single.tbd.PfTbd(config);
            initState = trueState(1, :);
            tic;
            pfAlgo.run(measData, initState, psfKernel);
            pfTime = toc;
            [pfPosRmse, pfVelRmse] = pfAlgo.computeRmse(trueState);
            
            fprintf('\n%-15s %12s %12s %12s\n', '算法', '执行时间[s]', '平均位置RMSE', '平均速度RMSE');
            fprintf('%-15s %12.4f %12.4f %12s\n', 'DP-TBD', dpTime, mean(dpPosRmse), 'N/A');
            fprintf('%-15s %12.4f %12.4f %12.4f\n', 'PF-TBD', pfTime, mean(pfPosRmse), mean(pfVelRmse));
            fprintf('\n');
        end
    end
end
