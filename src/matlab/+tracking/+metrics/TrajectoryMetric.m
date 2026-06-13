classdef TrajectoryMetric < handle
    % TRAJECTORYMETRIC 轨迹度量
    %
    % 实现轨迹级别的性能评估
    %
    % 使用示例:
    %   metric = tracking.metrics.TrajectoryMetric('c', 10, 'p', 2);
    %   [squaredGOSPA, gospaLoc, gospaMis, gospaFal] = tracking.metrics.compute(...
    %       X_estimate, t_b_estimate, length_estimate, ...
    %       X_truth, t_birth, t_death, k_end, Nx);
    %

    properties
        p       (1,1) double = 2      % 指数
        c       (1,1) double = 10     % 截断距离
        alpha   (1,1) double = 2      % 基数惩罚因子
        Results = struct()  % 计算结果
    end
    
    methods
        function obj = TrajectoryMetric(varargin)
            % TRAJECTORYMETRIC 构造函数
            %
            % 输入:
            %   varargin - 可选参数
            %     'p' - 指数
            %     'c' - 截断距离
            %     'alpha' - 基数惩罚因子
            
            % 设置默认值
            obj.p = 2;
            obj.c = 10;
            obj.alpha = 2;
            
            % 解析参数
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                
                switch lower(key)
                    case 'p'
                        obj.p = value;
                    case 'c'
                        obj.c = value;
                    case 'alpha'
                        obj.alpha = value;
                end
            end
        end
        
        function [squaredGOSPA, gospaLoc, gospaMis, gospaFal] = compute(obj, X_estimate, t_b_estimate, length_estimate, X_truth, t_birth, t_death, k_end, Nx)
            % COMPUTE 计算轨迹级GOSPA误差
            %
            % 输入:
            %   X_estimate - 估计轨迹集合
            %   t_b_estimate - 估计轨迹的出生时间
            %   length_estimate - 估计轨迹的长度
            %   X_truth - 真实轨迹状态
            %   t_birth - 真实轨迹的出生时间
            %   t_death - 真实轨迹的死亡时间
            %   k_end - 计算的结束时间
            %   Nx - 状态维度
            %
            % 输出:
            %   squaredGOSPA - 平方GOSPA误差
            %   gospaLoc - 定位误差
            %   gospaMis - 漏检误差
            %   gospaFal - 虚警误差
            
            % 验证输入
            if ~isnumeric(k_end) || k_end < 1
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_VALUE, ...
                    'k_end必须是正整数');
                throw(ME);
            end
            
            if ~isnumeric(Nx) || Nx < 1
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_VALUE, ...
                    'Nx必须是正整数');
                throw(ME);
            end
            
            % 初始化结果
            squaredGOSPA = 0;
            gospaLoc = 0;
            gospaMis = 0;
            gospaFal = 0;
            
            % 计算每个时间步的GOSPA
            for k = 1:k_end
                % 获取当前时间的真实目标
                aliveTargetsIndex = and(and(and(k >= t_birth, k < t_death), ...
                    k_end >= t_birth), k_end < t_death);
                nAliveTargets = sum(aliveTargetsIndex);
                
                % 提取真实目标位置
                if nAliveTargets > 0
                    aliveTargetsPosIndex = [aliveTargetsIndex; false(1, length(t_birth)); ...
                        aliveTargetsIndex; false(1, length(t_birth))];
                    Xk = reshape(X_truth(aliveTargetsPosIndex(:), k), 2, nAliveTargets);
                else
                    Xk = zeros(2, 0);
                end
                
                % 获取当前时间的估计目标
                aliveEstimateIndex = find(and(k >= t_b_estimate, ...
                    k <= t_b_estimate + length_estimate - 1));
                Yk = zeros(Nx, length(aliveEstimateIndex));
                
                for i = 1:length(aliveEstimateIndex)
                    indexLoc = k - t_b_estimate(aliveEstimateIndex(i)) + 1;
                    Yk(:, i) = X_estimate{aliveEstimateIndex(i)}((indexLoc-1)*Nx + 1 : indexLoc*Nx);
                end
                
                % 移除估计中的NaN值
                indexNonan = ~isnan(Yk(1, :));
                YkPos = Yk([1 3], indexNonan);
                
                % 计算GOSPA
                [dGospa, ~, decompCost] = tracking.metrics.GOSPA.run(Xk, YkPos, ...
                    'p', obj.p, 'c', obj.c, 'alpha', obj.alpha);
                
                % 累加结果
                squaredGOSPA = squaredGOSPA + dGospa^2;
                gospaLoc = gospaLoc + decompCost.localisation;
                gospaMis = gospaMis + decompCost.missed;
                gospaFal = gospaFal + decompCost.false;
            end
            
            % 归一化
            squaredGOSPA = squaredGOSPA / k_end;
            gospaLoc = gospaLoc / k_end;
            gospaMis = gospaMis / k_end;
            gospaFal = gospaFal / k_end;
            
            % 保存结果
            obj.Results = struct('squaredGOSPA', squaredGOSPA, 'gospaLoc', gospaLoc, 'gospaMis', gospaMis, 'gospaFal', gospaFal, 'kEnd', k_end, 'parameters', struct('p', obj.p, 'c', obj.c, 'alpha', obj.alpha));
        end
        
        function displayResults(obj)
            % DISPLAYRESULTS 显示结果
            
            if isempty(fieldnames(obj.Results))
                fprintf('没有结果可显示\n');
                return;
            end
            
            fprintf('\n===== 轨迹度量结果 =====\n');
            fprintf('平方GOSPA: %.4f\n', obj.Results.squaredGOSPA);
            fprintf('定位误差: %.4f\n', obj.Results.gospaLoc);
            fprintf('漏检误差: %.4f\n', obj.Results.gospaMis);
            fprintf('虚警误差: %.4f\n', obj.Results.gospaFal);
            fprintf('计算时间步: %d\n', obj.Results.kEnd);
            fprintf('参数: p=%.2f, c=%.2f, alpha=%.2f\n', ...
                obj.Results.parameters.p, ...
                obj.Results.parameters.c, ...
                obj.Results.parameters.alpha);
            fprintf('====================\n\n');
        end
    end
    
    methods (Static)
        function [squaredGOSPA, gospaLoc, gospaMis, gospaFal] = run(X_estimate, t_b_estimate, length_estimate, X_truth, t_birth, t_death, k_end, Nx, varargin)
            % RUN 静态方法 - 计算轨迹级GOSPA误差
            %
            % 输入:
            %   X_estimate - 估计轨迹集合
            %   t_b_estimate - 估计轨迹的出生时间
            %   length_estimate - 估计轨迹的长度
            %   X_truth - 真实轨迹状态
            %   t_birth - 真实轨迹的出生时间
            %   t_death - 真实轨迹的死亡时间
            %   k_end - 计算的结束时间
            %   Nx - 状态维度
            %   varargin - 可选参数
            %
            % 输出:
            %   squaredGOSPA - 平方GOSPA误差
            %   gospaLoc - 定位误差
            %   gospaMis - 漏检误差
            %   gospaFal - 虚警误差
            
            trajMetric = tracking.metrics.TrajectoryMetric(varargin{:});
            [squaredGOSPA, gospaLoc, gospaMis, gospaFal] = trajMetric.compute(...
                X_estimate, t_b_estimate, length_estimate, X_truth, t_birth, t_death, k_end, Nx);
        end
    end
end
