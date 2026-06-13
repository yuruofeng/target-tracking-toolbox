classdef PoissonComponent
    % POISSONCOMPONENT 泊松点过程分量
    %
    % 实现PMBM滤波器中的泊松分量（PPP）
    %
    % 使用示例:
    %   poisson = tracking.multi.rfs.pmbm.PoissonComponent(config);
    %   poisson = poisson.predict();
    %   poisson = poisson.update(z, H, R);
    %

    properties
        Weights    double  = []  % 高斯分量权重
        Means      double  = []  % 高斯分量均值
        Covariances double  = []  % 高斯分量协方差
        Active     logical = []  % 活跃标志
    end
    
    properties (Access = private)
        Config
    end
    
    methods
        function obj = PoissonComponent(config)
            % POISSONCOMPONENT 构造函数
            %
            % 输入:
            %   config - FilterConfig对象
            
            if nargin < 1
                ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                    '必须提供config参数');
                throw(ME);
            end
            
            obj.Config = config;
        end
        
        function obj = initialize(obj)
            % INITIALIZE 初始化泊松分量
            %
            % 使用新生模型初始化
            
            birthModel = obj.Config.birthModel;
            
            if ~isempty(birthModel.means)
                numBirth = size(birthModel.means, 2);
                
                obj.Weights = birthModel.weights * birthModel.intensity;
                obj.Means = birthModel.means;
                
                if ndims(birthModel.covs) == 3
                    obj.Covariances = birthModel.covs;
                else
                    stateDim = size(birthModel.means, 1);
                    obj.Covariances = repmat(birthModel.covs, [1, 1, numBirth]);
                end
                
                obj.Active = true(1, numBirth);
            end
        end
        
        function obj = predict(obj)
            % PREDICT 预测泊松分量
            %
            % 实现PPP预测
            
            if isempty(obj.Weights)
                return;
            end
            
            F = obj.Config.motionModel.F;
            Q = obj.Config.motionModel.Q;
            pS = obj.Config.survivalProb;
            
            activeIdx = find(obj.Active);
            
            for i = 1:length(activeIdx)
                idx = activeIdx(i);
                
                obj.Means(:, idx) = F * obj.Means(:, idx);
                obj.Covariances(:, :, idx) = F * obj.Covariances(:, :, idx) * F' + Q;
                obj.Weights(idx) = pS * obj.Weights(idx);
            end
            
            obj = obj.addBirth();
        end
        
        function obj = update(obj, measurement, H, R, pD, lambdaC)
            % UPDATE 更新泊松分量
            %
            % 输入:
            %   measurement - 量测集合
            %   H      - 量测矩阵
            %   R      - 量测噪声协方差
            %   pD     - 检测概率
            %   lambdaC - 杂波强度

            if isempty(obj.Weights) || isempty(measurement)
                return;
            end

            activeIdx = find(obj.Active);
            numActive = length(activeIdx);
            numMeas = size(measurement, 2);
            
            newWeights = [];
            newMeans = [];
            newCovs = [];
            newActive = [];
            
            for i = 1:numActive
                idx = activeIdx(i);
                weight_miss = (1 - pD) * obj.Weights(idx);
                
                newWeights = [newWeights, weight_miss];
                newMeans = [newMeans, obj.Means(:, idx)];
                newCovs = cat(3, newCovs, obj.Covariances(:, :, idx));
                newActive = [newActive, true];
            end
            
            for j = 1:numMeas
                meas_j = measurement(:, j);

                for i = 1:numActive
                    idx = activeIdx(i);

                    [K, S, innov] = obj.computeKalmanGain(...
                        obj.Means(:, idx), obj.Covariances(:, :, idx), H, R, meas_j);
                    
                    likelihood = obj.computeLikelihood(innov, S);
                    weight_update = pD * obj.Weights(idx) * likelihood / lambdaC;
                    
                    mean_update = obj.Means(:, idx) + K * innov;
                    cov_update = obj.Covariances(:, :, idx) - K * S * K';
                    cov_update = (cov_update + cov_update') / 2;
                    
                    newWeights = [newWeights, weight_update];
                    newMeans = [newMeans, mean_update];
                    newCovs = cat(3, newCovs, cov_update);
                    newActive = [newActive, true];
                end
            end
            
            obj.Weights = newWeights;
            obj.Means = newMeans;
            obj.Covariances = newCovs;
            obj.Active = newActive;
        end
        
        function obj = prune(obj, threshold)
            % PRUNE 剪枝泊松分量
            %
            % 输入:
            %   threshold - 剪枝阈值
            
            if isempty(obj.Weights)
                return;
            end
            
            activeIdx = find(obj.Active);
            pruneMask = obj.Weights(activeIdx) >= threshold;
            activeIdx = activeIdx(pruneMask);
            
            obj.Weights = obj.Weights(activeIdx);
            obj.Means = obj.Means(:, activeIdx);
            
            if ~isempty(activeIdx)
                obj.Covariances = obj.Covariances(:, :, activeIdx);
            else
                obj.Covariances = [];
            end
            
            obj.Active = true(1, length(obj.Weights));
        end
        
        function obj = addBirth(obj)
            % ADDBIRTH 添加新生分量

            birthModel = obj.Config.birthModel;

            if isempty(birthModel.means)
                return;
            end

            numBirth = size(birthModel.means, 2);

            obj.Weights = [obj.Weights, birthModel.weights * birthModel.intensity];

            % Handle empty Means case
            if isempty(obj.Means)
                obj.Means = birthModel.means;
            else
                obj.Means = [obj.Means, birthModel.means];
            end

            if ndims(birthModel.covs) == 3
                obj.Covariances = cat(3, obj.Covariances, birthModel.covs);
            else
                for i = 1:numBirth
                    obj.Covariances = cat(3, obj.Covariances, birthModel.covs);
                end
            end

            obj.Active = [obj.Active, true(1, numBirth)];
        end
        
        function intensity = getIntensity(obj)
            % GETINTENSITY 获取总强度
            
            if isempty(obj.Weights)
                intensity = 0;
            else
                intensity = sum(obj.Weights(obj.Active));
            end
        end
    end
    
    methods (Access = protected)
        function [K, S, innov] = computeKalmanGain(obj, mean, cov, H, R, z)
            z_pred = H * mean;
            innov = z - z_pred;
            S = H * cov * H' + R;
            S = (S + S') / 2;
            K = cov * H' / S;
        end
        
        function likelihood = computeLikelihood(obj, innov, S)
            nz = length(innov);
            detS = det(S);
            if detS < eps
                likelihood = eps;
                return;
            end
            d2 = innov' * (S \ innov);
            if d2 > 700
                likelihood = eps;
                return;
            end
            likelihood = exp(-0.5 * d2) / sqrt((2 * pi)^nz * detS);
            if likelihood < eps
                likelihood = eps;
            end
        end
    end
end
