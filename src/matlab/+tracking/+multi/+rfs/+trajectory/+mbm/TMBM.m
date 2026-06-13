classdef TMBM < tracking.multi.rfs.core.BaseFilter
    % TMBM 轨迹多伯努利混合滤波器
    %
    % 实现轨迹级别的MBM滤波器，估计目标轨迹集合
    % TMBM是TPMBM的基础，不包含泊松分量
    %
    % 参考文献:
    %   A. F. Garcia-Fernandez, L. Svensson, J. L. Williams, Y. Xia and 
    %   K. Granstrom, "Trajectory Poisson Multi-Bernoulli Filters," 
    %   IEEE Transactions on Signal Processing, vol. 68, pp. 4933-4945, 2020.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   tmbmFilter = tracking.multi.rfs.trajectory.mbm.TMBM(config);
    %   result = tmbmFilter.run(measurements, groundTruth);
    %

    properties (Access = private)
        MBMComponent
        GlobalHypotheses
        GlobalHypWeights
    end
    
    properties (Access = public)
        EstimateType = 'alive'
    end
    
    methods
        function obj = TMBM(config)
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            obj.MBMComponent = struct(...
                'tracks', {{}}, ...
                'trackIds', [], ...
                'birthTimes', [] ...
            );
            obj.GlobalHypotheses = [];
            obj.GlobalHypWeights = [];
        end
        
        function obj = initialize(obj)
            obj.MBMComponent = struct(...
                'tracks', {{}}, ...
                'trackIds', [], ...
                'birthTimes', [] ...
            );
            obj.GlobalHypotheses = [];
            obj.GlobalHypWeights = [];
            obj.State = struct(...
                'numTracks', 0, ...
                'numHypotheses', 0 ...
            );
        end
        
        function obj = predict(obj)
            numTracks = length(obj.MBMComponent.tracks);
            
            for i = 1:numTracks
                track = obj.MBMComponent.tracks{i};
                numSTH = length(track.singleTargetHypotheses);
                
                for h = 1:numSTH
                    sth = track.singleTargetHypotheses{h};
                    numSteps = size(sth.states, 2);
                    
                    xPred = obj.Config.motionModel.F * sth.states(:, numSteps);
                    PPred = obj.Config.motionModel.F * sth.covariances{numSteps} * obj.Config.motionModel.F' + obj.Config.motionModel.Q;
                    
                    track.singleTargetHypotheses{h}.states = [sth.states, xPred];
                    track.singleTargetHypotheses{h}.covariances{numSteps + 1} = PPred;
                    track.singleTargetHypotheses{h}.existenceProb = sth.existenceProb * obj.Config.survivalProb;
                end
                
                obj.MBMComponent.tracks{i} = track;
            end
            
            obj.State.numTracks = length(obj.MBMComponent.tracks);
            obj.State.numHypotheses = length(obj.GlobalHypWeights);
        end
        
        function obj = update(obj, z)
            if isempty(z)
                return;
            end
            
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            pD = obj.Config.detectionProb;
            lambdaC = obj.Config.clutterRate / prod(obj.Config.surveillanceArea);
            
            [zGated, ~, ~] = obj.gating(z);
            
            obj = obj.updateMBMComponent(zGated, H, R, pD, lambdaC);
            obj = obj.updateGlobalHypotheses(zGated);
            obj = obj.pruneMBMComponent();
            
            obj.State.numTracks = length(obj.MBMComponent.tracks);
            obj.State.numHypotheses = length(obj.GlobalHypWeights);
        end
        
        function estimates = estimate(obj)
            estimates = struct();
            estimates.states = [];
            estimates.cardinality = 0;
            estimates.numTrajectories = 0;
            estimates.trajectories = {};
            estimates.trackIds = [];
            
            if isempty(obj.GlobalHypWeights)
                return;
            end
            
            [~, maxIdx] = max(obj.GlobalHypWeights);
            bestHyp = obj.GlobalHypotheses(maxIdx, :);
            
            activeTracks = find(bestHyp > 0);
            
            for i = 1:length(activeTracks)
                trackIdx = activeTracks(i);
                sthIdx = bestHyp(trackIdx);
                
                if sthIdx > 0 && trackIdx <= length(obj.MBMComponent.tracks)
                    track = obj.MBMComponent.tracks{trackIdx};
                    
                    if sthIdx <= length(track.singleTargetHypotheses)
                        sth = track.singleTargetHypotheses{sthIdx};
                        
                        traj = struct();
                        traj.states = sth.states;
                        traj.covariances = sth.covariances;
                        traj.existenceProb = sth.existenceProb;
                        traj.trackId = obj.MBMComponent.trackIds(trackIdx);
                        traj.birthTime = obj.MBMComponent.birthTimes(trackIdx);
                        
                        estimates.numTrajectories = estimates.numTrajectories + 1;
                        estimates.trajectories{estimates.numTrajectories} = traj;
                        estimates.trackIds(estimates.numTrajectories) = traj.trackId;
                        estimates.states = [estimates.states, sth.states(:, end)];
                    end
                end
            end

            estimates.cardinality = size(estimates.states, 2);
        end
        
        function obj = prune(obj)
            obj = obj.pruneMBMComponent();
        end
    end
    
    methods (Access = private)
        function obj = updateMBMComponent(obj, z, H, R, pD, lambdaC)
            numTracks = length(obj.MBMComponent.tracks);
            numMeasurements = size(z, 2);
            
            for i = 1:numTracks
                track = obj.MBMComponent.tracks{i};
                numSTH = length(track.singleTargetHypotheses);
                
                newSTH = cell(numSTH + numMeasurements, 1);
                
                for h = 1:numSTH
                    newSTH{h} = track.singleTargetHypotheses{h};
                    newSTH{h}.existenceProb = newSTH{h}.existenceProb * (1 - pD);
                end
                
                for m = 1:numMeasurements
                    zM = z(:, m);
                    
                    for h = 1:numSTH
                        sth = track.singleTargetHypotheses{h};
                        numSteps = size(sth.states, 2);
                        
                        xPred = sth.states(:, numSteps);
                        PPred = sth.covariances{numSteps};
                        
                        S = H * PPred * H' + R;
                        K = PPred * H' / S;
                        zInnov = zM - H * xPred;
                        
                        xUpd = xPred + K * zInnov;
                        PUpd = PPred - K * S * K';
                        
                        lik = exp(-0.5 * zInnov' * (S \ zInnov)) / sqrt(det(2 * pi * S));
                        
                        newSTHIdx = (m - 1) * numSTH + h + numSTH;
                        newSTH{newSTHIdx} = struct();
                        newSTH{newSTHIdx}.states = [sth.states, xUpd];
                        newSTH{newSTHIdx}.covariances = sth.covariances;
                        newSTH{newSTHIdx}.covariances{numSteps + 1} = PUpd;
                        newSTH{newSTHIdx}.existenceProb = sth.existenceProb * pD * lik / (lambdaC + sth.existenceProb * pD * lik);
                        newSTH{newSTHIdx}.detectionHistory = [sth.detectionHistory, m];
                    end
                end
                
                track.singleTargetHypotheses = newSTH;
                obj.MBMComponent.tracks{i} = track;
            end
        end
        
        function obj = updateGlobalHypotheses(obj, z)
            numTracks = length(obj.MBMComponent.tracks);
            numMeasurements = size(z, 2);
            
            if isempty(obj.GlobalHypotheses)
                birthModel = obj.Config.getBirthModel('single');

                for m = 1:numMeasurements
                    newTrack = struct();
                    newTrack.singleTargetHypotheses = cell(1, 1);

                    sth = struct();
                    sth.states = birthModel.mean;
                    sth.covariances = {birthModel.cov};
                    sth.existenceProb = birthModel.existProb;
                    sth.detectionHistory = m;
                    
                    newTrack.singleTargetHypotheses{1} = sth;
                    
                    obj.MBMComponent.tracks{end+1} = newTrack;
                    obj.MBMComponent.trackIds(end+1) = max([0, obj.MBMComponent.trackIds]) + 1;
                    obj.MBMComponent.birthTimes(end+1) = obj.CurrentTime;
                end
                
                obj.GlobalHypotheses = ones(1, length(obj.MBMComponent.tracks));
                obj.GlobalHypWeights = 1;
                return;
            end
            
            newGlobalHyps = [];
            newGlobalHypWeights = [];
            
            numGlobalHyps = size(obj.GlobalHypotheses, 1);
            
            for g = 1:numGlobalHyps
                globalHyp = obj.GlobalHypotheses(g, :);
                globalHypWeight = obj.GlobalHypWeights(g);
                
                newGlobalHyps = [newGlobalHyps; globalHyp];
                newGlobalHypWeights = [newGlobalHypWeights; globalHypWeight * (1 - pD)^numMeasurements];
                
                for m = 1:numMeasurements
                    newHyp = globalHyp;
                    newHyp = [newHyp, 1];
                    newGlobalHyps = [newGlobalHyps; newHyp];
                    newGlobalHypWeights = [newGlobalHypWeights; globalHypWeight * pD / numMeasurements];
                end
            end
            
            obj.GlobalHypotheses = newGlobalHyps;
            obj.GlobalHypWeights = newGlobalHypWeights;
            
            totalWeight = sum(obj.GlobalHypWeights);
            if totalWeight > 0
                obj.GlobalHypWeights = obj.GlobalHypWeights / totalWeight;
            end
        end
        
        function obj = pruneMBMComponent(obj)
            weightThreshold = obj.Config.pruningThreshold;
            maxHypotheses = obj.Config.maxComponents;
            
            keepIdx = obj.GlobalHypWeights >= weightThreshold;
            
            if sum(keepIdx) > maxHypotheses
                [~, sortIdx] = sort(obj.GlobalHypWeights, 'descend');
                keepIdx = false(size(obj.GlobalHypWeights));
                keepIdx(sortIdx(1:maxHypotheses)) = true;
            end
            
            obj.GlobalHypotheses = obj.GlobalHypotheses(keepIdx, :);
            obj.GlobalHypWeights = obj.GlobalHypWeights(keepIdx);
            
            totalWeight = sum(obj.GlobalHypWeights);
            if totalWeight > 0
                obj.GlobalHypWeights = obj.GlobalHypWeights / totalWeight;
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
            
            keepIdx = true(1, size(z, 2));
            
            for m = 1:size(z, 2)
                zM = z(:, m);
                minDist = inf;
                
                for i = 1:length(obj.MBMComponent.tracks)
                    track = obj.MBMComponent.tracks{i};
                    for h = 1:length(track.singleTargetHypotheses)
                        sth = track.singleTargetHypotheses{h};
                        numSteps = size(sth.states, 2);
                        xPred = sth.states(:, numSteps);
                        PPred = sth.covariances{numSteps};
                        
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
    end
    
    methods (Static)
        function result = run(config, measurements, groundTruth)
            filter = tracking.multi.rfs.trajectory.mbm.TMBM(config);
            result = filter.run(measurements, groundTruth);
        end
    end
end
