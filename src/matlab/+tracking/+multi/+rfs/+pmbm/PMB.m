classdef PMB < tracking.multi.rfs.core.BaseFilter
    % PMB 泊松多伯努利滤波器
    %
    % 实现PMB滤波器，是PMBM滤波器的简化版本
    % PMB滤波器在每次更新后将PMBM密度投影为PMB密度
    %
    % 参考文献:
    %   J. L. Williams, "Marginal multi-Bernoulli filters: RFS derivation 
    %   of MHT, JIPDA, and association-based member," IEEE Transactions on 
    %   Aerospace and Electronic Systems, vol. 51, no. 3, pp. 1664-1687, 2015.
    %
    %   A. F. Garcia-Fernandez, L. Svensson, J. L. Williams, Y. Xia and 
    %   K. Granstrom, "Trajectory Poisson Multi-Bernoulli Filters," 
    %   IEEE Transactions on Signal Processing, vol. 68, pp. 4933-4945, 2020.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   pmbFilter = tracking.multi.rfs.pmbm.PMB(config);
    %   result = pmbFilter.run(measurements, groundTruth);
    %

    properties (Access = private)
        PoissonComponent
        MBComponent
        ProjectionThreshold = 0.001
    end
    
    methods
        function obj = PMB(config)
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            obj.PoissonComponent = tracking.multi.rfs.pmbm.PoissonComponent(config);
            obj.MBComponent.means = {};
            obj.MBComponent.covs = {};
            obj.MBComponent.existProbs = [];
            obj.MBComponent.trackIds = [];
            obj.MBComponent.birthTimes = [];
        end
        
        function obj = initialize(obj)
            obj.PoissonComponent = obj.PoissonComponent.initialize();
            obj.MBComponent.means = {};
            obj.MBComponent.covs = {};
            obj.MBComponent.existProbs = [];
            obj.MBComponent.trackIds = [];
            obj.MBComponent.birthTimes = [];
            obj.State = struct(...
                'poissonIntensity', obj.PoissonComponent.getIntensity(), ...
                'numTracks', 0 ...
            );
        end
        
        function obj = predict(obj)
            obj.PoissonComponent = obj.PoissonComponent.predict();
            for i = 1:length(obj.MBComponent.means)
                for j = 1:length(obj.MBComponent.means{i})
                    obj.MBComponent.means{i}{j} = obj.Config.motionModel.F * obj.MBComponent.means{i}{j};
                    obj.MBComponent.covs{i}{j} = obj.Config.motionModel.F * obj.MBComponent.covs{i}{j} * obj.Config.motionModel.F' + obj.Config.motionModel.Q;
                end
                obj.MBComponent.existProbs(i) = obj.MBComponent.existProbs(i) * obj.Config.survivalProb;
            end
            obj.State.poissonIntensity = obj.PoissonComponent.getIntensity();
            obj.State.numTracks = length(obj.MBComponent.existProbs);
        end
        
        function obj = update(obj, measurement)
            if isempty(measurement)
                return;
            end
            
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            pD = obj.Config.detectionProb;
            lambdaC = obj.Config.clutterRate / prod(obj.Config.surveillanceArea);
            
            [zGated, ~, ~] = obj.gating(measurement);
            
            obj.PoissonComponent = obj.PoissonComponent.update(zGated, H, R, pD, lambdaC);
            obj.PoissonComponent = obj.PoissonComponent.prune(obj.Config.pruningThreshold);
            
            obj = obj.updateMBComponent(zGated, H, R, pD, lambdaC);
            obj = obj.projectToPMB();
            obj = obj.pruneMBComponent();
            obj = obj.createTracksFromPoisson(zGated, H, R, pD, lambdaC);
            
            obj.State.poissonIntensity = obj.PoissonComponent.getIntensity();
            obj.State.numTracks = length(obj.MBComponent.existProbs);
        end
        
        function estimates = estimate(obj)
            % Extract actual estimates from MBComponent
            estimates = struct('states', [], 'weights', [], 'cardinality', 0);

            numTracks = length(obj.MBComponent.existProbs);
            if numTracks == 0
                return;
            end

            states = [];
            weights = [];

            % For each track, select the hypothesis with highest existence probability
            for i = 1:numTracks
                trackMeans = obj.MBComponent.means{i};
                trackExistProbs = obj.MBComponent.existProbs;

                if isempty(trackMeans)
                    continue;
                end

                % Find the hypothesis with maximum weight for this track
                % In PMB, after projection, there's typically one hypothesis per track
                % but we select the first one as the estimate
                if trackExistProbs(i) > obj.Config.existenceThreshold
                    states = [states, trackMeans{1}];
                    weights = [weights, trackExistProbs(i)];
                end
            end

            estimates.states = states;
            estimates.weights = weights;
            estimates.cardinality = size(states, 2);
        end
        
        function obj = prune(obj)
            obj.PoissonComponent = obj.PoissonComponent.prune(obj.Config.pruningThreshold);
            obj = obj.pruneMBComponent();
        end
    end
    
    methods (Access = private)
        function obj = updateMBComponent(obj, z, H, R, pD, lambdaC)
            numMeasurements = size(z, 2);
            numTracks = length(obj.MBComponent.existProbs);
            
            if numTracks == 0
                return;
            end
            
            newMeans = cell(numTracks, 1);
            newCovs = cell(numTracks, 1);
            newExistProbs = zeros(numTracks, 1);
            
            for i = 1:numTracks
                trackMeans = obj.MBComponent.means{i};
                trackCovs = obj.MBComponent.covs{i};
                numHyps = length(trackMeans);
                
                hypMeans = cell(numHyps + numMeasurements, 1);
                hypCovs = cell(numHyps + numMeasurements, 1);
                hypExistProbs = zeros(numHyps + numMeasurements, 1);
                
                r_i = obj.MBComponent.existProbs(i);
                for j = 1:numHyps
                    hypMeans{j} = trackMeans{j};
                    hypCovs{j} = trackCovs{j};
                    hypExistProbs(j) = r_i * (1 - pD);
                end
                
                for m = 1:numMeasurements
                    zM = z(:, m);
                    for j = 1:numHyps
                        xPred = trackMeans{j};
                        PPred = trackCovs{j};
                        
                        S = H * PPred * H' + R;
                        S = (S + S') / 2;
                        K = PPred * H' / S;
                        zInnov = zM - H * xPred;
                        
                        hypMeans{numHyps + m} = xPred + K * zInnov;
                        hypCovs{numHyps + m} = PPred - K * S * K';
                        
                        lik = obj.computeLikelihood(zInnov, S);
                        denom = lambdaC + r_i * pD * lik;
                        if denom > eps
                            hypExistProbs(numHyps + m) = r_i * pD * lik / denom;
                        else
                            hypExistProbs(numHyps + m) = 0;
                        end
                    end
                end
                
                newMeans{i} = hypMeans;
                newCovs{i} = hypCovs;
                newExistProbs(i) = min(sum(hypExistProbs), 1);
            end
            
            obj.MBComponent.means = newMeans;
            obj.MBComponent.covs = newCovs;
            obj.MBComponent.existProbs = newExistProbs;
        end
        
        function obj = projectToPMB(obj)
            numTracks = length(obj.MBComponent.existProbs);
            
            for i = 1:numTracks
                numHyps = length(obj.MBComponent.means{i});
                if numHyps <= 1
                    continue;
                end
                
                existProbs = zeros(numHyps, 1);
                for j = 1:numHyps
                    if j == 1
                        existProbs(j) = obj.MBComponent.existProbs(i) * (1 - obj.Config.detectionProb);
                    else
                        existProbs(j) = obj.MBComponent.existProbs(i) / numHyps * obj.Config.detectionProb;
                    end
                end
                
                totalExist = sum(existProbs);
                if totalExist > obj.ProjectionThreshold
                    weights = existProbs / totalExist;
                    
                    mergedMean = zeros(size(obj.MBComponent.means{i}{1}));
                    mergedCov = zeros(size(obj.MBComponent.covs{i}{1}));
                    
                    for j = 1:numHyps
                        mergedMean = mergedMean + weights(j) * obj.MBComponent.means{i}{j};
                    end
                    
                    for j = 1:numHyps
                        diff = obj.MBComponent.means{i}{j} - mergedMean;
                        mergedCov = mergedCov + weights(j) * (obj.MBComponent.covs{i}{j} + diff * diff');
                    end
                    
                    obj.MBComponent.means{i} = {mergedMean};
                    obj.MBComponent.covs{i} = {mergedCov};
                    obj.MBComponent.existProbs(i) = totalExist;
                end
            end
        end
        
        function obj = pruneMBComponent(obj)
            existThreshold = obj.Config.existenceThreshold;
            keepIdx = obj.MBComponent.existProbs >= existThreshold;
            
            obj.MBComponent.means = obj.MBComponent.means(keepIdx);
            obj.MBComponent.covs = obj.MBComponent.covs(keepIdx);
            obj.MBComponent.existProbs = obj.MBComponent.existProbs(keepIdx);
            obj.MBComponent.trackIds = obj.MBComponent.trackIds(keepIdx);
            obj.MBComponent.birthTimes = obj.MBComponent.birthTimes(keepIdx);
        end
        
        function obj = createTracksFromPoisson(obj, z, H, R, pD, lambdaC)
            numMeasurements = size(z, 2);
            
            for m = 1:numMeasurements
                zM = z(:, m);
                
                poissonIntensity = obj.PoissonComponent.getIntensity();
                
                if poissonIntensity < eps
                    continue;
                end
                
                birthSingle = obj.Config.getBirthModel('single');
                birthMean = birthSingle.mean;
                birthCov = birthSingle.cov;
                
                S = H * birthCov * H' + R;
                zPred = H * birthMean;
                zInnov = zM - zPred;
                maha = zInnov' / S * zInnov;
                
                if maha > obj.Config.gatingThreshold
                    continue;
                end
                
                K = birthCov * H' / S;
                updatedMean = birthMean + K * zInnov;
                updatedCov = birthCov - K * S * K';
                
                newTrackId = max([0, obj.MBComponent.trackIds]) + 1;
                
                obj.MBComponent.means{end+1} = {updatedMean};
                obj.MBComponent.covs{end+1} = {updatedCov};
                obj.MBComponent.existProbs(end+1) = birthSingle.existProb;
                obj.MBComponent.trackIds(end+1) = newTrackId;
                obj.MBComponent.birthTimes(end+1) = obj.CurrentTime;
            end
        end
        
        function [zGated, indices, distances] = gating(obj, z)
            zGated = z;
            indices = 1:size(z, 2);
            distances = zeros(1, size(z, 2));
            
            if isempty(z)
                return;
            end
            
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            gateThreshold = obj.Config.gatingThreshold;
            
            numTracks = length(obj.MBComponent.means);
            
            if numTracks == 0
                zGated = z;
                indices = 1:size(z, 2);
                distances = zeros(1, size(z, 2));
                return;
            end
            
            keepIdx = true(1, size(z, 2));
            
            for m = 1:size(z, 2)
                zM = z(:, m);
                minDist = inf;
                
                for i = 1:length(obj.MBComponent.means)
                    for j = 1:length(obj.MBComponent.means{i})
                        xPred = obj.MBComponent.means{i}{j};
                        PPred = obj.MBComponent.covs{i}{j};
                        
                        S = H * PPred * H' + R;
                        zInnov = zM - H * xPred;
                        dist = zInnov' * (S \ zInnov);
                        
                        if dist < minDist
                            minDist = dist;
                        end
                    end
                end
                
                distances(m) = minDist;
                if minDist > gateThreshold
                    keepIdx(m) = false;
                end
            end
            
            zGated = z(:, keepIdx);
            indices = find(keepIdx);
            distances = distances(keepIdx);
        end
        
        function lik = computeLikelihood(obj, innov, S)
            nz = length(innov);
            detS = det(2 * pi * S);
            if detS < eps
                lik = eps;
                return;
            end
            d2 = innov' * (S \ innov);
            if d2 > 700
                lik = eps;
                return;
            end
            lik = exp(-0.5 * d2) / sqrt(detS);
            if lik < eps
                lik = eps;
            end
        end
    end
end
