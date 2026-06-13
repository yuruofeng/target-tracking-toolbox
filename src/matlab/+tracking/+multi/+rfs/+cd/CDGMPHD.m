classdef CDGMPHD < tracking.multi.rfs.core.BaseFilter
    % CDGMPHD 连续-离散高斯混合PHD滤波器
    %
    % 实现连续-离散场景下的PHD滤波器
    %
    % 参考文献:
    %   A. F. García-Fernández, S. Maskell, "Continuous-discrete multiple 
    %   target filtering: PMBM, PHD and CPHD filter implementations," 
    %   IEEE Transactions on Signal Processing, vol. 68, pp. 1300-1314, 2020.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   filter = tracking.multi.rfs.cd.CDGMPHD(config);
    %   result = filter.run(measurements, groundTruth);
    %

    properties (Access = private)
        GaussianComponents       % 高斯分量
        MaxComponents = 30       % 最大分量数
        PruningThreshold = 1e-5  % 剪枝阈值
        MergingThreshold = 0.1   % 合并阈值
    end
    
    properties (Access = public)
        BirthParameters          % 新生参数
    end
    
    methods
        function obj = CDGMPHD(config)
            % CDGMPHD 构造函数
            %
            % 输入:
            %   config - FilterConfig对象
            
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            
            obj.GaussianComponents = struct(...
                'weights', [], ...
                'means', [], ...
                'covariances', [], ...
                'active', [] ...
            );
        end
        
        function obj = initialize(obj)
            % INITIALIZE 初始化滤波器
            
            birthModel = obj.Config.birthModel;
            
            if ~isempty(birthModel.means)
                numBirth = size(birthModel.means, 2);
                
                obj.GaussianComponents.weights = birthModel.weights * birthModel.intensity;
                obj.GaussianComponents.means = birthModel.means;
                
                if ndims(birthModel.covs) == 3
                    obj.GaussianComponents.covariances = birthModel.covs;
                else
                    stateDim = size(birthModel.means, 1);
                    obj.GaussianComponents.covariances = repmat(birthModel.covs, [1, 1, numBirth]);
                end
                
                obj.GaussianComponents.active = true(1, numBirth);
            end
        end
        
        function obj = predict(obj, deltaTime)
            % PREDICT 预测步骤
            %
            % 输入:
            %   deltaTime - 时间间隔
            
            if nargin < 2
                deltaTime = 1;
            end
            
            motionModel = obj.Config.motionModel;
            F = motionModel.F;
            Q = motionModel.Q;
            survivalProb = obj.Config.survivalProb;
            
            activeIdx = obj.GaussianComponents.active;
            
            obj.GaussianComponents.weights(activeIdx) = ...
                obj.GaussianComponents.weights(activeIdx) * survivalProb;
            
            for i = find(activeIdx)
                obj.GaussianComponents.means(:, i) = F * obj.GaussianComponents.means(:, i);
                obj.GaussianComponents.covariances(:, :, i) = ...
                    F * obj.GaussianComponents.covariances(:, :, i) * F' + Q;
            end
            
            obj = obj.addBirth(deltaTime);
        end
        
        function obj = update(obj, measurements)
            % UPDATE 更新步骤
            %
            % 输入:
            %   measurements - 测量集合
            
            if isempty(measurements)
                return;
            end
            
            detectionProb = obj.Config.detectionProb;
            measurementModel = obj.Config.measurementModel;
            H = measurementModel.H;
            R = measurementModel.R;
            clutterIntensity = obj.Config.clutterIntensity;
            
            activeIdx = find(obj.GaussianComponents.active);
            numComponents = length(activeIdx);
            numMeasurements = size(measurements, 2);
            
            if numComponents == 0
                return;
            end
            
            for i = 1:numMeasurements
                z = measurements(:, i);
                
                for j = 1:numComponents
                    idx = activeIdx(j);
                    m = obj.GaussianComponents.means(:, idx);
                    P = obj.GaussianComponents.covariances(:, :, idx);
                    
                    zPred = H * m;
                    S = H * P * H' + R;
                    K = P * H' / S;
                    
                    innovation = z - zPred;
                    likelihood = 1 / sqrt((2*pi)^length(z) * det(S)) * ...
                        exp(-0.5 * innovation' / S * innovation);
                    
                    eta = detectionProb * likelihood / ...
                        (detectionProb * likelihood + clutterIntensity);
                    
                    obj.GaussianComponents.weights(idx) = ...
                        obj.GaussianComponents.weights(idx) * (1 - eta);
                    
                    if eta > 0.01
                        newWeight = obj.GaussianComponents.weights(idx) * eta / (1 - eta);
                        newMean = m + K * innovation;
                        newCov = (eye(size(P)) - K * H) * P;
                        
                        obj = obj.addComponent(newWeight, newMean, newCov);
                    end
                end
            end
        end
        
        function estimates = estimate(obj)
            % ESTIMATE 状态估计
            %
            % 输出:
            %   estimates - 估计的目标状态
            
            estimates = [];
            activeIdx = find(obj.GaussianComponents.active);
            
            for i = activeIdx
                if obj.GaussianComponents.weights(i) > 0.5
                    estimates = [estimates, obj.GaussianComponents.means(:, i)];
                end
            end
        end
        
        function obj = prune(obj)
            % PRUNE 剪枝和合并
            
            activeIdx = find(obj.GaussianComponents.active);
            
            keepIdx = obj.GaussianComponents.weights(activeIdx) > obj.PruningThreshold;
            activeIdx = activeIdx(keepIdx);
            
            obj.GaussianComponents.active = false(1, length(obj.GaussianComponents.weights));
            obj.GaussianComponents.active(activeIdx) = true;
            
            if sum(obj.GaussianComponents.active) > obj.MaxComponents
                [~, sortIdx] = sort(obj.GaussianComponents.weights, 'descend');
                keepIdx = sortIdx(1:obj.MaxComponents);
                
                obj.GaussianComponents.active = false(1, length(obj.GaussianComponents.weights));
                obj.GaussianComponents.active(keepIdx) = true;
            end
        end
        
        function obj = addBirth(obj, deltaTime)
            % ADDBIRTH 添加新生分量
            
            birthModel = obj.Config.birthModel;
            
            if isempty(birthModel.means)
                return;
            end
            
            meanB = birthModel.means(:, 1);
            covB = birthModel.covs(:, :, 1);
            
            inactiveIdx = find(~obj.GaussianComponents.active, 1);
            
            if ~isempty(inactiveIdx)
                obj.GaussianComponents.weights(inactiveIdx) = birthModel.intensity;
                obj.GaussianComponents.means(:, inactiveIdx) = meanB;
                obj.GaussianComponents.covariances(:, :, inactiveIdx) = covB;
                obj.GaussianComponents.active(inactiveIdx) = true;
            end
        end
        
        function obj = addComponent(obj, weight, mean, cov)
            % ADDCOMPONENT 添加新分量
            
            inactiveIdx = find(~obj.GaussianComponents.active, 1);
            
            if ~isempty(inactiveIdx)
                obj.GaussianComponents.weights(inactiveIdx) = weight;
                obj.GaussianComponents.means(:, inactiveIdx) = mean;
                obj.GaussianComponents.covariances(:, :, inactiveIdx) = cov;
                obj.GaussianComponents.active(inactiveIdx) = true;
            end
        end
        
        function params = computeBirthParameters(obj, q, deltaTime, meanA, covA, mu)
            % COMPUTEBIRTHPARAMETERS 计算连续-离散新生参数
            %
            % 根据连续-离散模型计算最佳高斯新生参数
            %
            % 输入:
            %   q - 过程噪声强度
            %   deltaTime - 时间间隔
            %   meanA - 出现时刻均值
            %   covA - 出现时刻协方差
            %   mu - 死亡率参数
            
            d = length(meanA) / 2;
            meanA_p = meanA(1:2:end);
            meanA_v = meanA(2:2:end);
            P_a_pp = covA(1:2:end, 1:2:end);
            P_a_vv = covA(2:2:end, 2:2:end);
            P_a_pv = covA(1:2:end, 2:2:end);
            
            expTerm = exp(-mu * deltaTime);
            E_t = 1/mu - deltaTime * expTerm / (1 - expTerm);
            E_t2 = 1/(1-expTerm) * (2/mu^2 - expTerm * (deltaTime^2 + 2*deltaTime/mu + 2/mu^2));
            E_t3 = 1/(1-expTerm) * (6/mu^3 - expTerm * (deltaTime^3 + 3*deltaTime^2/mu + 6*deltaTime/mu^2 + 6/mu^3));
            C_t = E_t2 - E_t^2;
            
            meanB = [eye(d), E_t*eye(d); zeros(d), eye(d)] * [meanA_p; meanA_v];
            
            P_b_pp = C_t * (meanA_v * meanA_v') + q * E_t3/3 * eye(d) + ...
                     P_a_pp + E_t * (P_a_pv + P_a_pv') + E_t2 * P_a_vv;
            P_b_pv = q * E_t2/2 * eye(d) + P_a_pv + E_t * P_a_vv;
            P_b_vv = q * E_t * eye(d) + P_a_vv;
            
            params.mean = zeros(2*d, 1);
            params.mean(1:2:end) = meanB(1:d);
            params.mean(2:2:end) = meanB(d+1:end);
            
            params.cov = zeros(2*d);
            params.cov(1:2:end, 1:2:end) = P_b_pp;
            params.cov(1:2:end, 2:2:end) = P_b_pv;
            params.cov(2:2:end, 1:2:end) = P_b_pv';
            params.cov(2:2:end, 2:2:end) = P_b_vv;
        end
    end
    
    methods (Static)
        function result = run(config, measurements, groundTruth)
            % RUN 静态运行方法
            %
            % 输入:
            %   config - 配置对象
            %   measurements - 测量数据
            %   groundTruth - 真值数据
            %
            % 输出:
            %   result - FilterResult对象
            
            filter = tracking.multi.rfs.cd.CDGMPHD(config);
            filter = filter.initialize();
            
            numSteps = length(measurements);
            estimates = cell(1, numSteps);
            
            for k = 1:numSteps
                filter = filter.predict();
                filter = filter.update(measurements{k});
                estimates{k} = filter.estimate();
                filter = filter.prune();
            end
            
            result = tracking.multi.rfs.core.FilterResult();
            result.estimates = estimates;
            result.groundTruth = groundTruth;
        end
    end
end
