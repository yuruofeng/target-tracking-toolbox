classdef CDPMBM < tracking.multi.rfs.core.BaseFilter
    % CDPMBM 连续-离散泊松多伯努利混合滤波器
    %
    % 实现连续-离散场景下的PMBM滤波器
    %
    % 参考文献:
    %   A. F. García-Fernández, S. Maskell, "Continuous-discrete multiple 
    %   target filtering: PMBM, PHD and CPHD filter implementations," 
    %   IEEE Transactions on Signal Processing, vol. 68, pp. 1300-1314, 2020.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   filter = tracking.multi.rfs.cd.CDPMBM(config);
    %   result = filter.run(measurements, groundTruth);
    %

    properties (Access = private)
        PoissonComponent         % 泊松分量
        MBMComponent             % 多伯努利混合分量
        PruningThreshold = 1e-4  % 剪枝阈值
        PoissonPruningThreshold = 1e-5  % 泊松剪枝阈值
        MaxHypotheses = 200      % 最大假设数
        GatingThreshold = 20     % 波门阈值
        ExistenceThreshold = 1e-5  % 存在阈值
    end
    
    methods
        function obj = CDPMBM(config)
            % CDPMBM 构造函数
            
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            
            obj.PoissonComponent = struct(...
                'intensity', 0, ...
                'means', [], ...
                'covariances', [] ...
            );
            
            obj.MBMComponent = struct(...
                'tracks', {{}}, ...
                'globalHypotheses', [], ...
                'globalHypothesisWeights', [] ...
            );
        end
        
        function obj = initialize(obj)
            % INITIALIZE 初始化滤波器
            
            birthModel = obj.Config.birthModel;
            
            if ~isempty(birthModel.means)
                obj.PoissonComponent.intensity = birthModel.intensity;
                obj.PoissonComponent.means = birthModel.means;
                
                if ndims(birthModel.covs) == 3
                    obj.PoissonComponent.covariances = birthModel.covs;
                else
                    stateDim = size(birthModel.means, 1);
                    numComponents = size(birthModel.means, 2);
                    obj.PoissonComponent.covariances = repmat(birthModel.covs, [1, 1, numComponents]);
                end
            end
            
            obj.MBMComponent.tracks = {};
            obj.MBMComponent.globalHypotheses = [];
            obj.MBMComponent.globalHypothesisWeights = [];
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
            
            if ~isempty(obj.PoissonComponent.means)
                numComponents = size(obj.PoissonComponent.means, 2);
                for i = 1:numComponents
                    obj.PoissonComponent.means(:, i) = F * obj.PoissonComponent.means(:, i);
                    obj.PoissonComponent.covariances(:, :, i) = ...
                        F * obj.PoissonComponent.covariances(:, :, i) * F' + Q;
                end
                obj.PoissonComponent.intensity = pS * obj.PoissonComponent.intensity;
            end
            
            for i = 1:length(obj.MBMComponent.tracks)
                track = obj.MBMComponent.tracks{i};
                for j = 1:length(track.hypotheses)
                    obj.MBMComponent.tracks{i}.hypotheses{j}.mean = ...
                        F * track.hypotheses{j}.mean;
                    obj.MBMComponent.tracks{i}.hypotheses{j}.covariance = ...
                        F * track.hypotheses{j}.covariance * F' + Q;
                    obj.MBMComponent.tracks{i}.hypotheses{j}.existence = ...
                        pS * track.hypotheses{j}.existence;
                end
            end
            
            birthModel = obj.Config.birthModel;
            if ~isempty(birthModel.means)
                if isfield(obj.Config.extraParams, 'deathRate')
                    deathRate = obj.Config.extraParams.deathRate;
                else
                    deathRate = 0.01;
                end
                birthIntensity = birthModel.intensity * (1 - exp(-deathRate * deltaTime));
                
                if isempty(obj.PoissonComponent.means)
                    obj.PoissonComponent.means = birthModel.means;
                    obj.PoissonComponent.covariances = birthModel.covs;
                else
                    obj.PoissonComponent.means = [obj.PoissonComponent.means, birthModel.means];
                    obj.PoissonComponent.covariances = cat(3, obj.PoissonComponent.covariances, birthModel.covs);
                end
                obj.PoissonComponent.intensity = obj.PoissonComponent.intensity + birthIntensity;
            end
        end
        
        function obj = update(obj, measurements)
            % UPDATE 更新步骤
            
            if isempty(measurements)
                return;
            end
            
            pD = obj.Config.detectionProb;
            clutterIntensity = obj.Config.clutterIntensity;
            
            if ~isempty(obj.PoissonComponent.means)
                obj = obj.updatePoissonWithMeasurements(measurements, pD, clutterIntensity);
            end
            
            obj = obj.updateMBMWithMeasurements(measurements, pD, clutterIntensity);
            
            obj = obj.pruneComponents();
        end
        
        function estimates = estimate(obj)
            % ESTIMATE 状态估计
            
            estimates = [];
            
            for i = 1:length(obj.MBMComponent.tracks)
                track = obj.MBMComponent.tracks{i};
                bestHypIdx = 1;
                bestWeight = 0;
                
                for j = 1:length(track.hypotheses)
                    if track.hypotheses{j}.existence > bestWeight
                        bestWeight = track.hypotheses{j}.existence;
                        bestHypIdx = j;
                    end
                end
                
                if bestWeight > 0.4
                    estimates = [estimates, track.hypotheses{bestHypIdx}.mean];
                end
            end
        end
        
        function obj = prune(obj)
            % PRUNE 剪枝
            
            obj = obj.pruneComponents();
        end
        
        function obj = pruneComponents(obj)
            % PRUNECOMPONENTS 剪枝所有分量
            
            if ~isempty(obj.PoissonComponent.means)
                intensities = obj.PoissonComponent.intensity / size(obj.PoissonComponent.means, 2);
                keepIdx = intensities > obj.PoissonPruningThreshold;
                
                obj.PoissonComponent.means = obj.PoissonComponent.means(:, keepIdx);
                obj.PoissonComponent.covariances = obj.PoissonComponent.covariances(:, :, keepIdx);
            end
            
            keepTracks = false(1, length(obj.MBMComponent.tracks));
            for i = 1:length(obj.MBMComponent.tracks)
                track = obj.MBMComponent.tracks{i};
                keepHypotheses = false(1, length(track.hypotheses));
                
                for j = 1:length(track.hypotheses)
                    if track.hypotheses{j}.existence > obj.ExistenceThreshold
                        keepHypotheses(j) = true;
                    end
                end
                
                obj.MBMComponent.tracks{i}.hypotheses = track.hypotheses(keepHypotheses);
                
                if ~isempty(obj.MBMComponent.tracks{i}.hypotheses)
                    keepTracks(i) = true;
                end
            end
            
            obj.MBMComponent.tracks = obj.MBMComponent.tracks(keepTracks);
            
            if length(obj.MBMComponent.tracks) > obj.MaxHypotheses
                [~, sortIdx] = sort(obj.MBMComponent.globalHypothesisWeights, 'descend');
                keepIdx = sortIdx(1:obj.MaxHypotheses);
                
                obj.MBMComponent.globalHypotheses = obj.MBMComponent.globalHypotheses(keepIdx, :);
                obj.MBMComponent.globalHypothesisWeights = obj.MBMComponent.globalHypothesisWeights(keepIdx);
            end
        end
    end
    
    methods (Access = private)
        function obj = updatePoissonWithMeasurements(obj, measurements, pD, clutterIntensity)
            % UPDATEPOISSONWITHMEASUREMENTS 使用测量更新泊松分量
            
            numMeasurements = size(measurements, 2);
            numPoissonComponents = size(obj.PoissonComponent.means, 2);
            
            newTracks = {};
            
            for j = 1:numMeasurements
                z = measurements(:, j);
                
                for i = 1:numPoissonComponents
                    m = obj.PoissonComponent.means(:, i);
                    P = obj.PoissonComponent.covariances(:, :, i);
                    
                    [zPred, S] = obj.predictMeasurement(m, P);
                    innov = z - zPred;
                    
                    mahalanobisDist = innov' * inv(S) * innov;
                    
                    if mahalanobisDist < obj.GatingThreshold
                        K = P * obj.Config.measurementModel.H' / S;
                        newMean = m + K * innov;
                        newCov = (eye(size(P)) - K * obj.Config.measurementModel.H) * P;
                        
                        likelihood = exp(-0.5 * mahalanobisDist) / sqrt(det(2*pi*S));
                        newExistence = pD * likelihood * obj.PoissonComponent.intensity / numPoissonComponents;
                        newExistence = newExistence / (newExistence + clutterIntensity);
                        
                        newTrack = struct();
                        newTrack.hypotheses = {struct('mean', newMean, 'covariance', newCov, 'existence', newExistence)};
                        newTracks{end+1} = newTrack;
                    end
                end
            end
            
            if ~isempty(newTracks)
                obj.MBMComponent.tracks = [obj.MBMComponent.tracks, newTracks];
            end
        end
        
        function [zPred, S] = predictMeasurement(obj, mean, cov)
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            zPred = H * mean;
            S = H * cov * H' + R;
        end

        function obj = updateMBMWithMeasurements(obj, measurements, pD, clutterIntensity)
            % UPDATEMBMWITHMEASUREMENTS 使用测量更新MBM分量
            
            for i = 1:length(obj.MBMComponent.tracks)
                track = obj.MBMComponent.tracks{i};
                newHypotheses = {};
                
                for j = 1:length(track.hypotheses)
                    hyp = track.hypotheses{j};
                    misdetectionHyp = hyp;
                    misdetectionHyp.existence = hyp.existence * (1 - pD);
                    newHypotheses{end+1} = misdetectionHyp;
                    
                    for k = 1:size(measurements, 2)
                        z = measurements(:, k);
                        
                        [zPred, S] = obj.predictMeasurement(hyp.mean, hyp.covariance);
                        innov = z - zPred;
                        
                        mahalanobisDist = innov' * inv(S) * innov;
                        
                        if mahalanobisDist < obj.GatingThreshold
                            K = hyp.covariance * obj.Config.measurementModel.H' / S;
                            newMean = hyp.mean + K * innov;
                            newCov = (eye(size(hyp.covariance)) - K * obj.Config.measurementModel.H) * hyp.covariance;
                            
                            likelihood = exp(-0.5 * mahalanobisDist) / sqrt(det(2*pi*S));
                            newExistence = hyp.existence * pD * likelihood / (hyp.existence * pD * likelihood + clutterIntensity);
                            
                            detectionHyp = struct('mean', newMean, 'covariance', newCov, 'existence', newExistence);
                            newHypotheses{end+1} = detectionHyp;
                        end
                    end
                end
                
                obj.MBMComponent.tracks{i}.hypotheses = newHypotheses;
            end
        end
    end
    
    methods (Static)
        function result = run(config, measurements, groundTruth)
            % RUN 静态运行方法
            
            filter = tracking.multi.rfs.cd.CDPMBM(config);
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
