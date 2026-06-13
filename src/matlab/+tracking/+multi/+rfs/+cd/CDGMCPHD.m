classdef CDGMCPHD < tracking.multi.rfs.core.BaseFilter
    % CDGMCPHD 连续-离散高斯混合CPHD滤波器
    %
    % 实现连续-离散场景下的CPHD滤波器
    %
    % 参考文献:
    %   A. F. García-Fernández, S. Maskell, "Continuous-discrete multiple 
    %   target filtering: PMBM, PHD and CPHD filter implementations," 
    %   IEEE Transactions on Signal Processing, vol. 68, pp. 1300-1314, 2020.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   filter = tracking.multi.rfs.cd.CDGMCPHD(config);
    %   result = filter.run(measurements, groundTruth);
    %

    properties (Access = private)
        GaussianComponents       % 高斯分量
        Cardinality              % 基数分布
        MaxComponents = 30       % 最大分量数
        MaxCardinality = 15      % 最大基数
        PruningThreshold = 1e-5  % 剪枝阈值
        MergingThreshold = 0.1   % 合并阈值
    end
    
    methods
        function obj = CDGMCPHD(config)
            % CDGMCPHD 构造函数
            
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            
            obj.GaussianComponents = struct(...
                'weights', [], ...
                'means', [], ...
                'covariances', [], ...
                'active', [] ...
            );
            
            obj.Cardinality = zeros(obj.MaxCardinality + 1, 1);
            obj.Cardinality(1) = 1;
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
                
                lambda = birthModel.intensity;
                n = (0:obj.MaxCardinality)';
                obj.Cardinality = exp(-lambda) * (lambda.^n) ./ factorial(n);
            end
        end
        
        function obj = predict(obj, deltaTime)
            % PREDICT 预测步骤
            
            if nargin < 2
                deltaTime = 1;
            end
            
            motionModel = obj.Config.motionModel;
            F = motionModel.F;
            Q = motionModel.Q;
            pS = obj.Config.survivalProb;
            
            activeIdx = find(obj.GaussianComponents.active);
            
            for i = activeIdx
                obj.GaussianComponents.means(:, i) = F * obj.GaussianComponents.means(:, i);
                obj.GaussianComponents.covariances(:, :, i) = ...
                    F * obj.GaussianComponents.covariances(:, :, i) * F' + Q;
                obj.GaussianComponents.weights(i) = pS * obj.GaussianComponents.weights(i);
            end
            
            birthModel = obj.Config.birthModel;
            if ~isempty(birthModel.means)
                numBirth = size(birthModel.means, 2);
                for j = 1:numBirth
                    obj = obj.addComponent(birthModel.weights(j) * birthModel.intensity, ...
                                          birthModel.means(:, j), ...
                                          birthModel.covs(:, :, j));
                end
            end
            
            obj.Cardinality = obj.predictCardinality(obj.Cardinality, pS, birthModel.intensity);
        end
        
        function obj = update(obj, measurements)
            % UPDATE 更新步骤
            
            if isempty(measurements)
                return;
            end
            
            pD = obj.Config.detectionProb;
            clutterIntensity = obj.Config.clutterIntensity;
            
            activeIdx = find(obj.GaussianComponents.active);
            numComponents = length(activeIdx);
            numMeasurements = size(measurements, 2);
            
            totalWeight = sum(obj.GaussianComponents.weights(activeIdx));
            
            for i = activeIdx
                m = obj.GaussianComponents.means(:, i);
                P = obj.GaussianComponents.covariances(:, :, i);
                w = obj.GaussianComponents.weights(i);
                
                for j = 1:numMeasurements
                    z = measurements(:, j);
                    
                    [zPred, S] = obj.predictMeasurement(m, P);
                    K = P * obj.Config.measurementModel.H' / S;
                    innov = z - zPred;
                    
                    likelihood = exp(-0.5 * innov' * inv(S) * innov) / sqrt(det(2*pi*S));
                    
                    newWeight = w * pD * likelihood / clutterIntensity;
                    newMean = m + K * innov;
                    newCov = (eye(size(P)) - K * obj.Config.measurementModel.H) * P;
                    
                    obj = obj.addComponent(newWeight, newMean, newCov);
                end
            end
        end
        
        function estimates = estimate(obj)
            % ESTIMATE 状态估计
            
            estimates = [];
            activeIdx = find(obj.GaussianComponents.active);
            
            meanCardinality = sum((0:obj.MaxCardinality)' .* obj.Cardinality);
            
            totalWeight = sum(obj.GaussianComponents.weights(activeIdx));
            
            if totalWeight > 0
                numEstimates = round(meanCardinality);
                [~, sortIdx] = sort(obj.GaussianComponents.weights, 'descend');
                
                for i = 1:min(numEstimates, length(sortIdx))
                    idx = sortIdx(i);
                    if obj.GaussianComponents.active(idx) && ...
                       obj.GaussianComponents.weights(idx) > 0.5
                        estimates = [estimates, obj.GaussianComponents.means(:, idx)];
                    end
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
            
            meanCard = sum((0:obj.MaxCardinality)' .* obj.Cardinality);
            totalWeight = sum(obj.GaussianComponents.weights(obj.GaussianComponents.active));
            
            if totalWeight > 0
                obj.GaussianComponents.weights = obj.GaussianComponents.weights / totalWeight * meanCard;
            end
        end
        
        function obj = addComponent(obj, weight, mean, cov)
            inactiveIdx = find(~obj.GaussianComponents.active, 1);

            if ~isempty(inactiveIdx)
                obj.GaussianComponents.weights(inactiveIdx) = weight;
                obj.GaussianComponents.means(:, inactiveIdx) = mean;
                obj.GaussianComponents.covariances(:, :, inactiveIdx) = cov;
                obj.GaussianComponents.active(inactiveIdx) = true;
            else
                obj.GaussianComponents.weights(end + 1) = weight;
                obj.GaussianComponents.means(:, end + 1) = mean;
                obj.GaussianComponents.covariances(:, :, end + 1) = cov;
                obj.GaussianComponents.active(end + 1) = true;
            end
        end

        function [zPred, S] = predictMeasurement(obj, mean, cov)
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            zPred = H * mean;
            S = H * cov * H' + R;
        end

        function cardPred = predictCardinality(obj, cardCurr, pS, lambdaBirth)
            % PREDICTCARDINALITY 预测基数分布
            
            nMax = obj.MaxCardinality;
            cardPred = zeros(nMax + 1, 1);
            
            for n = 0:nMax
                for m = n:nMax
                    cardPred(n + 1) = cardPred(n + 1) + ...
                        nchoosek(m, n) * pS^n * (1-pS)^(m-n) * cardCurr(m + 1);
                end
                
                cardBirth = exp(-lambdaBirth) * lambdaBirth^n / factorial(n);
                cardPred(n + 1) = cardPred(n + 1) + cardBirth;
            end
            
            cardPred = cardPred / sum(cardPred);
        end
    end
    
    methods (Static)
        function result = run(config, measurements, groundTruth)
            % RUN 静态运行方法
            
            filter = tracking.multi.rfs.cd.CDGMCPHD(config);
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
