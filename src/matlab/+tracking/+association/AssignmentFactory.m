classdef AssignmentFactory
    % ASSIGNMENTFACTORY 数据关联算法工厂类
    %
    % 使用工厂模式创建不同类型的数据关联算法
    %
    % 使用示例:
    %   factory = tracking.association.AssignmentFactory();
    %   algorithm = factory.createAlgorithm('Murty', 10);  % 前10个最佳分配
    %   [assignments, costs] = algorithm.solve(costMatrix);
    %

    properties (Constant)
        % 支持的算法类型
        AlgorithmTypes = struct(...
            'MURTY', 'Murty', ...
            'AUCTION', 'Auction', ...
            'MUNKRES', 'Munkres' ...
        )
    end
    
    methods (Static)
        function algorithm = createAlgorithm(algorithmType, varargin)
            % CREATEALGORITHM 创建数据关联算法
            %
            % 输入:
            %   algorithmType - 算法类型字符串
            %   varargin - 算法特定参数
            %
            % 输出:
            %   algorithm - 算法对象
            
            % 验证输入
            if nargin < 1
                ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                    '必须提供algorithmType参数');
                throw(ME);
            end
            
            if ~ischar(algorithmType) && ~isstring(algorithmType)
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_TYPE, ...
                    'algorithmType必须是字符串');
                throw(ME);
            end
            
            % 根据类型创建算法
            switch upper(algorithmType)
                case {'MURTY', 'K_BEST', 'TOP_K'}
                    if length(varargin) >= 1
                        kBest = varargin{1};
                        algorithm = tracking.association.Murty(kBest);
                    else
                        algorithm = tracking.association.Murty(1);  % 默认只返回最佳分配
                    end
                    
                case {'AUCTION', 'AUCTION_ALGORITHM'}
                    algorithm = tracking.association.Auction(varargin{:});
                    
                case {'MUNKRES', 'HUNGARIAN', 'OPTIMAL'}
                    algorithm = tracking.association.Munkres();
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '不支持的算法类型: %s', algorithmType);
                    throw(ME);
            end
        end
        
        function types = getAvailableTypes()
            % GETAVAILABLETYPES 获取可用的算法类型
            %
            % 输出:
            %   types - 可用算法类型列表
            
            types = {'Murty', 'Auction', 'Munkres'};
        end
        
        function displayAvailableTypes()
            % DISPLAYAVAILABLETYPES 显示可用的算法类型
            
            fprintf('\n===== 可用的数据关联算法 =====\n');
            types = tracking.association.AssignmentFactory.getAvailableTypes();
            for i = 1:length(types)
                fprintf('%d. %s\n', i, types{i});
            end
            fprintf('=============================\n\n');
        end
        
        function [assignment, cost] = solve(costMatrix, algorithmType, varargin)
            % SOLVE 直接求解分配问题
            %
            % 输入:
            %   costMatrix   - 成本矩阵
            %   algorithmType - 算法类型
            %   varargin     - 算法参数
            %
            % 输出:
            %   assignment - 分配结果
            %   cost - 最优成本
            
            algorithm = tracking.association.AssignmentFactory.createAlgorithm(algorithmType, varargin{:});
            [assignment, cost] = algorithm.solve(costMatrix);
        end
        
        function algorithm = createFromPreset(presetName, varargin)
            % CREATEFROMPRESET 从预设创建算法
            %
            % 输入:
            %   presetName - 预设名称
            %   varargin   - 额外参数
            %
            % 输出:
            %   algorithm - 算法对象
            
            switch lower(presetName)
                case 'k_best'
                    if length(varargin) < 1
                        kBest = 5;
                    else
                        kBest = varargin{1};
                    end
                    algorithm = tracking.association.AssignmentFactory.createAlgorithm('Murty', kBest);
                    
                case 'fast'
                    algorithm = tracking.association.AssignmentFactory.createAlgorithm('Auction', 'MaxIterations', 100);
                    
                case 'optimal'
                    algorithm = tracking.association.AssignmentFactory.createAlgorithm('Munkres');
                    
                otherwise
                    ME = tracking.core.MTTException(tracking.core.ErrorCode.UNSUPPORTED_OPERATION, ...
                        '未知的预设名称: %s', presetName);
                    throw(ME);
            end
        end
    end
end
