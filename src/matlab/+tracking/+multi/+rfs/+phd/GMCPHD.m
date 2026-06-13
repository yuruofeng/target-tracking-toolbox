classdef GMCPHD < tracking.multi.rfs.core.BaseFilter
    % GMCPHD 高斯混合基数概率假设密度滤波器
    %
    % 实现基于高斯混合表示的CPHD滤波器，同时估计目标数和状态
    %
    % 参考文献:
    %   B.-T. Vo, B.-N. Vo, and A. Cantoni, "Analytic implementations of the
    %   cardinalized probability hypothesis density filter," IEEE Trans. Signal 
    %   Process., vol. 55, no. 7, pp. 3553-3567, Jul. 2007.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   cphdFilter = tracking.multi.rfs.phd.GMCPHD(config);
    %   result = cphdFilter.run(measurements, groundTruth);
    %

    properties (Access = private)
        GaussianComponents  % 高斯分量存储
        Cardinality         % 基数分布
        MaxCardinality      % 最大基数
    end
    
    methods
        function obj = GMCPHD(config)
            % GMCPHD 构造函数
            %
            % 输入:
            %   config - FilterConfig对象
            
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            
            % 设置最大基数
            obj.MaxCardinality = 20;
            
            % 初始化高斯分量
            obj.GaussianComponents = struct(...
                'weights', [], ...
                'means', [], ...
                'covariances', [], ...
                'active', [] ...
            );
            
            % 初始化基数分布
            obj.Cardinality = zeros(obj.MaxCardinality + 1, 1);
            obj.Cardinality(1) = 1;  % 初始认为有0个目标
        end
        
        function obj = initialize(obj)
            % INITIALIZE 初始化滤波器
            %
            % 初始化PHD强度函数和基数分布
            
            % 使用新生模型初始化
            birthModel = obj.Config.birthModel;
            
            if ~isempty(birthModel.means)
                numBirth = size(birthModel.means, 2);
                
                obj.GaussianComponents.weights = birthModel.weights * birthModel.intensity;
                obj.GaussianComponents.means = birthModel.means;
                
                % 处理协方差矩阵
                if ndims(birthModel.covs) == 3
                    obj.GaussianComponents.covariances = birthModel.covs;
                else
                    stateDim = size(birthModel.means, 1);
                    obj.GaussianComponents.covariances = repmat(birthModel.covs, [1, 1, numBirth]);
                end
                
                obj.GaussianComponents.active = true(1, numBirth);
            end
            
            % 初始化基数分布（泊松分布）
            lambda0 = birthModel.intensity;
            for n = 0:obj.MaxCardinality
                obj.Cardinality(n + 1) = poisspdf(n, lambda0);
            end
            
            % 归一化
            obj.Cardinality = obj.Cardinality / sum(obj.Cardinality);
            
            % 初始化状态
            obj.State = struct(...
                'numComponents', length(obj.GaussianComponents.weights), ...
                'totalIntensity', sum(obj.GaussianComponents.weights), ...
                'expectedCardinality', obj.computeExpectedCardinality() ...
            );
        end
        
        function obj = predict(obj)
            % PREDICT 预测步骤
            %
            % 实现CPHD预测方程（强度和基数）
            
            components = obj.GaussianComponents;
            
            % 获取运动模型
            F = obj.Config.motionModel.F;
            Q = obj.Config.motionModel.Q;
            pS = obj.Config.survivalProb;
            
            % 预测存活目标的强度
            if ~isempty(components.weights)
                activeIdx = find(components.active);
                numActive = length(activeIdx);
                
                for i = 1:numActive
                    idx = activeIdx(i);
                    
                    % 状态预测
                    components.means(:, idx) = F * components.means(:, idx);
                    
                    % 协方差预测
                    components.covariances(:, :, idx) = ...
                        F * components.covariances(:, :, idx) * F' + Q;
                    
                    % 权重预测（乘以存活概率）
                    components.weights(idx) = pS * components.weights(idx);
                end
            end
            
            % 预测基数分布
            obj.Cardinality = obj.predictCardinality();
            
            % 添加新生目标
            obj = obj.addBirthComponents();
            
            % 更新高斯分量
            obj.GaussianComponents = components;
            
            % 更新状态
            obj.State.numComponents = sum(components.active);
            obj.State.totalIntensity = sum(components.weights(components.active));
            obj.State.expectedCardinality = obj.computeExpectedCardinality();
        end
        
        function obj = update(obj, z)
            % UPDATE 更新步骤
            %
            % 输入:
            %   z - 当前时刻的量测集合
            %
            % 实现CPHD更新方程
            
            components = obj.GaussianComponents;
            
            if isempty(components.weights)
                return;
            end
            
            % 获取量测模型
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            pD = obj.Config.detectionProb;
            lambdaC = obj.Config.clutterRate / prod(obj.Config.surveillanceArea);
            
            % 活跃分量索引
            activeIdx = find(components.active);
            numActive = length(activeIdx);
            numMeas = size(z, 2);
            
            % 存储更新后的分量
            newWeights = [];
            newMeans = [];
            newCovs = [];
            newActive = [];
            
            % 1. 漏检分量
            for i = 1:numActive
                idx = activeIdx(i);
                weight_miss = (1 - pD) * components.weights(idx);
                
                newWeights = [newWeights, weight_miss];
                newMeans = [newMeans, components.means(:, idx)];
                newCovs = cat(3, newCovs, components.covariances(:, :, idx));
                newActive = [newActive, true];
            end
            
            % 2. 检测分量
            likelihoods = zeros(numActive, numMeas);
            
            for j = 1:numMeas
                z_j = z(:, j);
                
                for i = 1:numActive
                    idx = activeIdx(i);
                    
                    % 计算卡尔曼增益和更新
                    [K, S, innov] = obj.computeKalmanGain(...
                        components.means(:, idx), ...
                        components.covariances(:, :, idx), ...
                        H, R, z_j);
                    
                    % 计算似然
                    likelihoods(i, j) = obj.computeLikelihood(innov, S);
                    
                    % 更新权重
                    weight_update = pD * components.weights(idx) * likelihoods(i, j);
                    
                    % 更新状态和协方差
                    mean_update = components.means(:, idx) + K * innov;
                    cov_update = components.covariances(:, :, idx) - K * S * K';
                    cov_update = (cov_update + cov_update') / 2;
                    
                    newWeights = [newWeights, weight_update];
                    newMeans = [newMeans, mean_update];
                    newCovs = cat(3, newCovs, cov_update);
                    newActive = [newActive, true];
                end
            end
            
            % 更新高斯分量
            obj.GaussianComponents.weights = newWeights;
            obj.GaussianComponents.means = newMeans;
            obj.GaussianComponents.covariances = newCovs;
            obj.GaussianComponents.active = newActive;
            
            % 更新基数分布
            obj.Cardinality = obj.updateCardinality(numMeas, likelihoods, lambdaC);
            
            % 更新状态
            obj.State.numComponents = length(newWeights);
            obj.State.totalIntensity = sum(newWeights);
            obj.State.expectedCardinality = obj.computeExpectedCardinality();
        end
        
        function estimate = estimate(obj)
            % ESTIMATE 状态估计
            %
            % 输出:
            %   estimate - 估计的目标状态集合
            
            estimate = struct('states', [], 'weights', [], 'cardinality', 0);
            
            if isempty(obj.GaussianComponents.weights)
                return;
            end
            
            % 使用基数分布估计目标数
            N_est = round(obj.State.expectedCardinality);
            estimate.cardinality = N_est;
            
            if N_est == 0
                return;
            end
            
            % 选择权重最大的N_est个分量
            activeWeights = obj.GaussianComponents.weights(obj.GaussianComponents.active);
            [~, sortedIdx] = sort(activeWeights, 'descend');
            
            % 获取活跃分量的索引
            activeIdx = find(obj.GaussianComponents.active);
            
            % 提取前N_est个状态
            selectedIdx = activeIdx(sortedIdx(1:min(N_est, length(sortedIdx))));
            
            estimate.states = obj.GaussianComponents.means(:, selectedIdx);
            estimate.weights = obj.GaussianComponents.weights(selectedIdx);
        end
        
        function obj = prune(obj)
            % PRUNE 剪枝和合并
            
            components = obj.GaussianComponents;
            
            if isempty(components.weights)
                return;
            end
            
            T_prune = obj.Config.pruningThreshold;
            T_merge = obj.Config.mergingThreshold;
            maxComponents = obj.Config.maxComponents;
            
            activeIdx = find(components.active);
            
            % 1. 剪枝
            pruneMask = components.weights(activeIdx) >= T_prune;
            activeIdx = activeIdx(pruneMask);
            
            % 2. 合并
            if ~isempty(activeIdx)
                [weights, means, covs, activeIdx] = obj.mergeComponents(...
                    components.weights(activeIdx), ...
                    components.means(:, activeIdx), ...
                    components.covariances(:, :, activeIdx), ...
                    T_merge, maxComponents);
                
                components.weights = weights;
                components.means = means;
                components.covariances = covs;
                components.active = true(1, length(weights));
            else
                components.weights = [];
                components.means = [];
                components.covariances = [];
                components.active = [];
            end
            
            obj.GaussianComponents = components;
            
            % 更新状态
            obj.State.numComponents = length(components.weights);
            obj.State.totalIntensity = sum(components.weights);
        end
    end
    
    methods (Access = protected)
        function cardinality_pred = predictCardinality(obj)
            % PREDICTCARDINALITY 预测基数分布
            
            pS = obj.Config.survivalProb;
            birthModel = obj.Config.birthModel;
            lambdaBirth = birthModel.intensity;
            
            % 新生基数分布（泊松）
            birthCard = zeros(obj.MaxCardinality + 1, 1);
            for n = 0:obj.MaxCardinality
                birthCard(n + 1) = poisspdf(n, lambdaBirth);
            end
            
            % 存活基数分布（二项分布卷积）
            cardinality_pred = zeros(obj.MaxCardinality + 1, 1);
            for n = 0:obj.MaxCardinality
                for j = 0:n
                    if j + 1 <= length(obj.Cardinality) && n - j + 1 <= length(birthCard)
                        % 存活目标的基数
                        pSurvive = binopdf(j, min(n, obj.MaxCardinality), pS);
                        cardinality_pred(n + 1) = cardinality_pred(n + 1) + ...
                            obj.Cardinality(j + 1) * birthCard(n - j + 1) * pSurvive;
                    end
                end
            end
            
            % 归一化
            if sum(cardinality_pred) > 0
                cardinality_pred = cardinality_pred / sum(cardinality_pred);
            end
        end
        
        function cardinality_upd = updateCardinality(obj, numMeas, likelihoods, lambdaC)
            % UPDATECARDINALITY 更新基数分布
            
            pD = obj.Config.detectionProb;
            numActive = size(likelihoods, 1);
            
            cardinality_upd = zeros(obj.MaxCardinality + 1, 1);
            
            for n = 0:obj.MaxCardinality
                % 计算基数n的似然
                if n < numMeas
                    % 不可能的情况：检测数超过目标数
                    L_n = 0;
                else
                    % 计算似然
                    L_n = obj.computeCardinalityLikelihood(n, numMeas, likelihoods, lambdaC, pD);
                end
                
                cardinality_upd(n + 1) = obj.Cardinality(n + 1) * L_n;
            end
            
            % 归一化
            if sum(cardinality_upd) > 0
                cardinality_upd = cardinality_upd / sum(cardinality_upd);
            end
        end
        
        function L_n = computeCardinalityLikelihood(obj, n, m, likelihoods, lambdaC, pD)
            % COMPUTECARDINALITYLIKELIHOOD 计算基数n的似然
            
            % 简化实现：使用泊松杂波模型
            if m == 0
                % 无量测
                L_n = (1 - pD)^n;
            else
                % 有量测：简化计算
                totalLikelihood = sum(likelihoods(:));
                L_n = (pD^m * (1 - pD)^(n - m)) * exp(-lambdaC) * ...
                      (lambdaC^(m) / factorial(m)) * (1 + totalLikelihood);
            end
        end
        
        function expectedCard = computeExpectedCardinality(obj)
            % COMPUTEEXPECTECDARDINALITY 计算期望基数
            
            expectedCard = 0;
            for n = 0:obj.MaxCardinality
                expectedCard = expectedCard + n * obj.Cardinality(n + 1);
            end
        end
        
        function obj = addBirthComponents(obj)
            % ADDBIRTHCOMPONENTS 添加新生分量
            
            birthModel = obj.Config.birthModel;
            
            if isempty(birthModel.means)
                return;
            end
            
            numBirth = size(birthModel.means, 2);
            
            obj.GaussianComponents.weights = [...
                obj.GaussianComponents.weights, ...
                birthModel.weights * birthModel.intensity];
            
            obj.GaussianComponents.means = [...
                obj.GaussianComponents.means, ...
                birthModel.means];
            
            if ndims(birthModel.covs) == 3
                obj.GaussianComponents.covariances = cat(3, ...
                    obj.GaussianComponents.covariances, ...
                    birthModel.covs);
            else
                for i = 1:numBirth
                    obj.GaussianComponents.covariances = cat(3, ...
                        obj.GaussianComponents.covariances, ...
                        birthModel.covs);
                end
            end
            
            obj.GaussianComponents.active = [...
                obj.GaussianComponents.active, ...
                true(1, numBirth)];
        end
        
        function [K, S, innov] = computeKalmanGain(obj, mean, cov, H, R, z)
            % COMPUTEKALMANGAIN 计算卡尔曼增益
            
            z_pred = H * mean;
            innov = z - z_pred;
            S = H * cov * H' + R;
            S = (S + S') / 2;
            K = cov * H' / S;
        end
        
        function likelihood = computeLikelihood(obj, innov, S)
            % COMPUTELIKELIHOOD 计算似然
            
            nz = length(innov);
            d2 = innov' * (S \ innov);
            likelihood = exp(-0.5 * d2) / sqrt((2 * pi)^nz * det(S));
        end
        
        function [weights, means, covs, activeIdx] = mergeComponents(obj, ...
                weights, means, covs, threshold, maxComponents)
            % MERGECOMPONENTS 合并相近的高斯分量
            
            numComponents = length(weights);
            
            if numComponents <= maxComponents
                activeIdx = 1:numComponents;
                return;
            end
            
            % 计算分量之间的距离
            distances = zeros(numComponents, numComponents);
            for i = 1:numComponents
                for j = i+1:numComponents
                    diff = means(:, i) - means(:, j);
                    distances(i, j) = sqrt(diff' * (covs(:, :, i) \ diff));
                    distances(j, i) = distances(i, j);
                end
            end
            
            % 合并距离小于阈值的分量
            merged = false(1, numComponents);
            newWeights = [];
            newMeans = [];
            newCovs = [];
            
            for i = 1:numComponents
                if merged(i)
                    continue;
                end
                
                closeIdx = find(distances(i, :) < threshold & ~merged);
                
                if length(closeIdx) > 1
                    merged(closeIdx) = true;
                    
                    w_sum = sum(weights(closeIdx));
                    w_normalized = weights(closeIdx) / w_sum;
                    
                    m_merged = means(:, closeIdx) * w_normalized';
                    P_merged = zeros(size(covs, 1), size(covs, 2));
                    
                    for k = 1:length(closeIdx)
                        idx_k = closeIdx(k);
                        diff = means(:, idx_k) - m_merged;
                        P_merged = P_merged + w_normalized(k) * ...
                            (covs(:, :, idx_k) + diff * diff');
                    end
                    
                    newWeights = [newWeights, w_sum];
                    newMeans = [newMeans, m_merged];
                    newCovs = cat(3, newCovs, P_merged);
                else
                    newWeights = [newWeights, weights(i)];
                    newMeans = [newMeans, means(:, i)];
                    newCovs = cat(3, newCovs, covs(:, :, i));
                end
            end
            
            if length(newWeights) > maxComponents
                [~, sortedIdx] = sort(newWeights, 'descend');
                keepIdx = sortedIdx(1:maxComponents);
                
                weights = newWeights(keepIdx);
                means = newMeans(:, keepIdx);
                covs = newCovs(:, :, keepIdx);
                activeIdx = keepIdx;
            else
                weights = newWeights;
                means = newMeans;
                covs = newCovs;
                activeIdx = 1:length(newWeights);
            end
        end
    end
end
