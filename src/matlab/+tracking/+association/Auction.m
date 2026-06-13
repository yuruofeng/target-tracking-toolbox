classdef Auction
    % AUCTION 拍卖算法实现
    %
    % 实现拍卖算法用于解决分配问题
    %
    % 参考文献:
    %   Section 6.5.1 in 'Design and Analysis of Modern Tracking
    %   Systems' by Blackman and Popoli, 1999 edition
    %
    % 使用示例:
    %   auction = tracking.association.Auction();
    %   [personToObj, objToPerson, cost] = auction.solve(costMatrix);
    %

    properties (Access = private)
        MaxIterations  = 1000  % 最大迭代次数
        Epsilon        = 0     % 出价步长
        UnassignedFlag = 0     % 未分配标记
    end
    
    properties (Access = public)
        Results = struct()  % 算法结果
    end
    
    methods
        function obj = Auction(varargin)
            % AUCTION 构造函数
            %
            % 输入:
            %   varargin - 可选参数
            %     'MaxIterations' - 最大迭代次数
            %     'Epsilon' - 出价步长
            
            % 设置默认值
            obj.MaxIterations = 1000;
            obj.Epsilon = 0;
            
            % 解析参数
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                
                switch lower(key)
                    case 'maxiterations'
                        obj.MaxIterations = value;
                    case 'epsilon'
                        obj.Epsilon = value;
                end
            end
        end
        
        function [personToObj, objToPerson, cost] = solve(obj, costMatrix)
            % SOLVE 解决分配问题
            %
            % 输入:
            %   costMatrix - 成本矩阵 (m x n)
            %
            % 输出:
            %   personToObj - 人员到对象的分配
            %   objToPerson - 对象到人员的分配
            %   cost - 最优成本
            
            % 验证输入
            if ~ismatrix(costMatrix) || size(costMatrix, 1) == 0 || size(costMatrix, 2) == 0
                ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_INPUT, ...
                    '成本矩阵必须是非空矩阵');
                throw(ME);
            end
            
            mPerson = size(costMatrix, 1);
            nObj = size(costMatrix, 2);
            
            % 处理特殊情况
            if mPerson == 1
                [cost, personToObj] = max(costMatrix);
                objToPerson = obj.UnassignedFlag * ones(nObj, 1);
                objToPerson(personToObj) = 1;
                return;
            end
            
            if nObj == 1
                [cost, objToPerson] = max(costMatrix);
                personToObj = obj.UnassignedFlag * ones(mPerson, 1);
                personToObj(objToPerson) = 1;
                return;
            end
            
            % 处理行数大于列数的情况
            swapDimFlag = false;
            if nObj < mPerson
                costMatrix = costMatrix';
                mPerson = size(costMatrix, 1);
                nObj = size(costMatrix, 2);
                swapDimFlag = true;
            end
            
            % 初始化
            personToObj = obj.UnassignedFlag * ones(mPerson, 1);
            objToPerson = obj.UnassignedFlag * ones(nObj, 1);
            cost = 0;
            pObj = zeros(1, nObj);  % 对象价格
            
            % 计算默认epsilon
            if obj.Epsilon == 0
                obj.Epsilon = 1 / max(nObj, mPerson);
            end
            
            % 主循环
            iter = 0;
            while ~all(personToObj ~= obj.UnassignedFlag)
                if iter > obj.MaxIterations
                    warning('Auction算法达到最大迭代次数');
                    break;
                end
                
                for i = 1:mPerson
                    if personToObj(i) == obj.UnassignedFlag
                        % 计算人员i对每个对象的价值
                        [valIJ, j] = sort(costMatrix(i, :) - pObj, 2, 'descend');
                        
                        % 最佳和次佳对象
                        jStar = j(1);
                        vIJStar = valIJ(1);
                        wIJStar = valIJ(2);
                        
                        % 出价
                        if wIJStar ~= -Inf
                            pObj(jStar) = pObj(jStar) + vIJStar - wIJStar + obj.Epsilon;
                        else
                            pObj(jStar) = pObj(jStar) + vIJStar + obj.Epsilon;
                        end
                        
                        % 处理对象已分配的情况
                        if objToPerson(jStar) ~= obj.UnassignedFlag
                            % 移除旧分配的成本
                            cost = cost - costMatrix(objToPerson(jStar), jStar);
                            personToObj(objToPerson(jStar)) = obj.UnassignedFlag;
                        end
                        
                        % 进行新分配
                        objToPerson(jStar) = i;
                        personToObj(i) = jStar;
                        cost = cost + costMatrix(i, jStar);
                    end
                end
                
                iter = iter + 1;
            end
            
            % 处理维度交换
            if swapDimFlag
                tmp = objToPerson;
                objToPerson = personToObj;
                personToObj = tmp;
            end
            
            % 保存结果
            obj.Results = struct('personToObj', personToObj, 'objToPerson', objToPerson, 'cost', cost, 'iterations', iter, 'converged', all(personToObj ~= obj.UnassignedFlag));
        end
        
        function displayResults(obj)
            % DISPLAYRESULTS 显示结果
            
            if isempty(fieldnames(obj.Results))
                fprintf('没有结果可显示\n');
                return;
            end
            
            fprintf('\n===== 拍卖算法结果 =====\n');
            fprintf('最优成本: %.4f\n', obj.Results.cost);
            fprintf('迭代次数: %d\n', obj.Results.iterations);
            if obj.Results.converged
                fprintf('收敛状态: 成功\n');
            else
                fprintf('收敛状态: 失败\n');
            end
            
            if ~isempty(obj.Results.personToObj)
                fprintf('人员分配: ');
                fprintf('%d ', obj.Results.personToObj);
                fprintf('\n');
            end
            
            if ~isempty(obj.Results.objToPerson)
                fprintf('对象分配: ');
                fprintf('%d ', obj.Results.objToPerson);
                fprintf('\n');
            end
            fprintf('====================\n\n');
        end
    end
    
    methods (Static)
        function [personToObj, objToPerson, cost] = run(costMatrix, varargin)
            % RUN 静态方法 - 解决分配问题
            %
            % 输入:
            %   costMatrix - 成本矩阵
            %   varargin - 可选参数
            %
            % 输出:
            %   personToObj - 人员到对象的分配
            %   objToPerson - 对象到人员的分配
            %   cost - 最优成本
            
            auction = tracking.association.Auction(varargin{:});
            [personToObj, objToPerson, cost] = auction.solve(costMatrix);
        end
    end
end
