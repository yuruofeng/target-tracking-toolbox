classdef GOSPA < handle
    % GOSPA 广义最优子模式分配度量
    %
    % 实现GOSPA度量用于评估多目标跟踪性能
    %
    % 参考文献:
    %   A. S. Rahmathullah, Á. F. García-Fernández and L. Svensson, 
    %   "Generalized optimal sub-pattern assignment metric", 
    %   2017 20th International Conference on Information Fusion (Fusion), 
    %   Xi'an, 2017, pp. 1-8.
    %
    % 使用示例:
    %   gospa = tracking.metrics.GOSPA('p', 2, 'c', 10, 'alpha', 2);
    %   [distance, assignment, decomposition] = gospa.compute(x, y);
    %

    properties
        p       (1,1) double = 2      % 指数 (1<=p<inf)
        c       (1,1) double = 10     % 截断距离 (c>0)
        alpha   (1,1) double = 2      % 基数惩罚因子 (0<alpha<=2)
        Results = struct()  % 计算结果
    end
    
    methods
        function obj = GOSPA(varargin)
            % GOSPA 构造函数
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
            
            % 验证参数
            obj.validateParameters();
        end
        
        function validateParameters(obj)
            % VALIDATEPARAMETERS 验证参数
            
            if ~((obj.p >= 1) && (obj.p < inf))
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_VALUE, ...
                    '指数p必须在[1,inf)范围内');
                throw(ME);
            end
            
            if ~(obj.c > 0)
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_VALUE, ...
                    '截断距离c必须大于0');
                throw(ME);
            end
            
            if ~((obj.alpha > 0) && (obj.alpha <= 2))
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_PARAMETER_VALUE, ...
                    'alpha必须在(0,2]范围内');
                throw(ME);
            end
        end
        
        function [distance, assignmentResult, decomposition] = compute(obj, x, y)
            % COMPUTE 计算GOSPA距离
            %
            % 输入:
            %   x - 估计目标集合 (状态维度 x 目标数)
            %   y - 真实目标集合 (状态维度 x 目标数)
            %
            % 输出:
            %   distance - GOSPA距离
            %   assignmentResult - 分配结果
            %   decomposition - 距离分解
            
            % ===== 输入规范化 =====
            % 处理两个空集的情况
            if isempty(x) && isempty(y)
                distance = 0;
                assignmentResult = [];
                decomposition = struct('localisation', 0, 'missed', 0, 'false', 0);
                obj.Results = struct('distance', distance, 'assignment', assignmentResult, ...
                    'decomposition', decomposition, 'numEstimated', 0, 'numTrue', 0, ...
                    'parameters', struct('p', obj.p, 'c', obj.c, 'alpha', obj.alpha));
                return;
            end
            
            % 确定状态维度(从非空的一方获取)
            state_dim = 0;
            if ~isempty(x)
                state_dim = size(x, 1);
            elseif ~isempty(y)
                state_dim = size(y, 1);
            end
            
            % 规范化空矩阵为正确维度
            if isempty(x)
                x = zeros(state_dim, 0);
            end
            if isempty(y)
                y = zeros(state_dim, 0);
            end
            % ===== 规范化结束 =====
            
            % 验证输入
            if size(x, 1) ~= size(y, 1)
                ME = tracking.core.MTTException(tracking.core.ErrorCode.DIMENSION_MISMATCH, ...
                    'x和y的行数必须相等');
                throw(ME);
            end
            
            nx = size(x, 2);  % x中的目标数
            ny = size(y, 2);  % y中的目标数
            
            % 初始化结果
            distance = 0;
            assignmentResult = [];
            decomposition = struct('localisation', 0, 'missed', 0, 'false', 0);
            
            % 计算成本矩阵
            costMatrix = zeros(nx, ny);
            for ix = 1:nx
                for iy = 1:ny
                    costMatrix(ix, iy) = min(obj.computeBaseDistance(x(:, ix), y(:, iy)), obj.c);
                end
            end
            
            dummyCost = (obj.c^obj.p) / obj.alpha;  % 基数不匹配的惩罚
            optCost = 0;
            
            if nx == 0
                % x为空，y中的所有目标都是false
                optCost = -ny * dummyCost;
                decomposition.false = optCost;
            else
                if ny == 0
                    % y为空，x中的所有目标都是missed
                    optCost = -nx * dummyCost;
                    if obj.alpha == 2
                        decomposition.missed = optCost;
                    end
                else
                    % 双方都非空，使用拍卖算法
                    costMatrix = -(costMatrix.^obj.p);
                    [xToYAssignment, yToXAssignment, ~] = tracking.association.Auction.run(costMatrix);

                    % 计算成本
                    for ind = 1:nx
                        if xToYAssignment(ind) ~= 0
                            optCost = optCost + costMatrix(ind, xToYAssignment(ind));

                            if obj.alpha == 2
                                if costMatrix(ind, xToYAssignment(ind)) > -obj.c^obj.p
                                    decomposition.localisation = decomposition.localisation + costMatrix(ind, xToYAssignment(ind));
                                else
                                    decomposition.missed = decomposition.missed - dummyCost;
                                    decomposition.false = decomposition.false - dummyCost;
                                end
                            end
                        else
                            optCost = optCost - dummyCost;
                            if obj.alpha == 2
                                decomposition.missed = decomposition.missed - dummyCost;
                            end
                        end
                    end

                    % 处理未分配的y目标
                    optCost = optCost - sum(yToXAssignment == 0) * dummyCost;
                    if obj.alpha == 2
                        decomposition.false = decomposition.false - sum(yToXAssignment == 0) * dummyCost;
                    end

                    % 将分配结果赋值给输出变量
                    assignmentResult = xToYAssignment;
                end
            end
            
            % 计算最终距离
            distance = (-optCost)^(1/obj.p);
            
            % 处理分解结果
            if obj.alpha == 2
                decomposition.localisation = -decomposition.localisation;
                decomposition.missed = -decomposition.missed;
                decomposition.false = -decomposition.false;
            end
            
            % 保存结果
            obj.Results = struct('distance', distance, 'assignment', assignmentResult, 'decomposition', decomposition, 'numEstimated', nx, 'numTrue', ny, 'parameters', struct('p', obj.p, 'c', obj.c, 'alpha', obj.alpha));
        end
        
        function displayResults(obj)
            % DISPLAYRESULTS 显示结果
            
            if isempty(fieldnames(obj.Results))
                fprintf('没有结果可显示\n');
                return;
            end
            
            fprintf('\n===== GOSPA度量结果 =====\n');
            fprintf('GOSPA距离: %.4f\n', obj.Results.distance);
            fprintf('估计目标数: %d\n', obj.Results.numEstimated);
            fprintf('真实目标数: %d\n', obj.Results.numTrue);
            fprintf('参数: p=%.2f, c=%.2f, alpha=%.2f\n', ...
                obj.Results.parameters.p, ...
                obj.Results.parameters.c, ...
                obj.Results.parameters.alpha);
            
            if obj.Results.parameters.alpha == 2
                fprintf('分解结果:\n');
                fprintf('  定位误差: %.4f\n', obj.Results.decomposition.localisation);
                fprintf('  漏检误差: %.4f\n', obj.Results.decomposition.missed);
                fprintf('  虚警误差: %.4f\n', obj.Results.decomposition.false);
            end
            
            if ~isempty(obj.Results.assignment)
                fprintf('分配结果: ');
                fprintf('%d ', obj.Results.assignment);
                fprintf('\n');
            end
            fprintf('====================\n\n');
        end
    end
    
    methods (Access = protected)
        function distance = computeBaseDistance(obj, xVec, yVec)
            % COMPUTEBASEDISTANCE 计算基础距离
            %
            % 输入:
            %   xVec - 第一个向量
            %   yVec - 第二个向量
            %
            % 输出:
            %   distance - 欧几里得距离
            
            distance = norm(xVec - yVec);
        end
    end
    
    methods (Static)
        function [distance, assignment, decomposition] = run(x, y, varargin)
            % RUN 静态方法 - 计算GOSPA距离
            %
            % 输入:
            %   x - 估计目标集合
            %   y - 真实目标集合
            %   varargin - 可选参数
            %
            % 输出:
            %   distance - GOSPA距离
            %   assignment - 分配结果
            %   decomposition - 距离分解
            
            gospa = tracking.metrics.GOSPA(varargin{:});
            [distance, assignment, decomposition] = gospa.compute(x, y);
        end
    end
end
