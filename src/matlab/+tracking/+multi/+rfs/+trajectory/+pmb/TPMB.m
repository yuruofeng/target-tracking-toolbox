classdef TPMB < tracking.multi.rfs.core.BaseFilter
    % TPMB 轨迹泊松多伯努利滤波器
    %
    % 实现轨迹级别的PMB滤波器，估计目标轨迹集合
    % 
    % 参考文献:
    %   A. F. Garcia-Fernandez, L. Svensson, J. L. Williams, Y. Xia and 
    %   K. Granstrom, "Trajectory Poisson Multi-Bernoulli Filters," 
    %   IEEE Transactions on Signal Processing, vol. 68, pp. 4933-4945, 2020.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   tpmbFilter = tracking.multi.rfs.trajectory.pmb.TPMB(config);
    %   result = tpmbFilter.run(measurements, groundTruth);
    %

    properties (Access = private)
        PoissonComponent
        TrajectoryComponents
        ProjectionThreshold = 0.001
    end
    
    properties (Access = public)
        EstimateType = 'alive'
    end
    
    methods
        function obj = TPMB(config)
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            obj.PoissonComponent = tracking.multi.rfs.pmbm.PoissonComponent(config);
            obj.TrajectoryComponents.trajectories = {};
            obj.TrajectoryComponents.existProbs = [];
            obj.TrajectoryComponents.trackIds = [];
            obj.TrajectoryComponents.birthTimes = [];
            obj.TrajectoryComponents.deathTimes = [];
        end
        
        function obj = initialize(obj)
            obj.PoissonComponent = obj.PoissonComponent.initialize();
            obj.TrajectoryComponents.trajectories = {};
            obj.TrajectoryComponents.existProbs = [];
            obj.TrajectoryComponents.trackIds = [];
            obj.TrajectoryComponents.birthTimes = [];
            obj.TrajectoryComponents.deathTimes = [];
            obj.State = struct(...
                'poissonIntensity', obj.PoissonComponent.getIntensity(), ...
                'numTrajectories', 0 ...
            );
        end
        
        function obj = predict(obj)
            obj.PoissonComponent = obj.PoissonComponent.predict();
            
            for i = 1:length(obj.TrajectoryComponents.trajectories)
                traj = obj.TrajectoryComponents.trajectories{i};
                numHyps = length(traj.hypotheses);
                
                for h = 1:numHyps
                    hyp = traj.hypotheses{h};
                    numSteps = size(hyp.states, 2);
                    
                    xPred = obj.Config.motionModel.F * hyp.states(:, numSteps);
                    
                    if length(hyp.covariances) < numSteps || isempty(hyp.covariances{numSteps})
                        PPred = obj.Config.motionModel.Q;
                    else
                        PPred = obj.Config.motionModel.F * hyp.covariances{numSteps} * obj.Config.motionModel.F' + obj.Config.motionModel.Q;
                    end
                    
                    traj.hypotheses{h}.states = [hyp.states, xPred];
                    traj.hypotheses{h}.covariances{numSteps + 1} = PPred;
                end
                
                obj.TrajectoryComponents.trajectories{i} = traj;
                obj.TrajectoryComponents.existProbs(i) = obj.TrajectoryComponents.existProbs(i) * obj.Config.survivalProb;
            end
            
            obj.State.poissonIntensity = obj.PoissonComponent.getIntensity();
            obj.State.numTrajectories = length(obj.TrajectoryComponents.existProbs);
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
            
            obj.PoissonComponent = obj.PoissonComponent.update(zGated, H, R, pD, lambdaC);
            obj.PoissonComponent = obj.PoissonComponent.prune(obj.Config.pruningThreshold);
            
            obj = obj.updateTrajectoryComponents(zGated, H, R, pD, lambdaC);
            obj = obj.projectToTPMB();
            obj = obj.pruneTrajectoryComponents();
            obj = obj.createTrajectoriesFromPoisson(zGated, H, R, pD, lambdaC);
            
            obj.State.poissonIntensity = obj.PoissonComponent.getIntensity();
            obj.State.numTrajectories = length(obj.TrajectoryComponents.existProbs);
        end
        
        function estimates = estimate(obj)
            % Extract actual estimates from TrajectoryComponents
            estimates = struct('states', [], 'weights', [], 'cardinality', 0);

            numTrajs = length(obj.TrajectoryComponents.existProbs);
            if numTrajs == 0
                return;
            end

            states = [];
            weights = [];

            % For each trajectory, extract current state estimate
            for i = 1:numTrajs
                traj = obj.TrajectoryComponents.trajectories{i};
                trajExistProb = obj.TrajectoryComponents.existProbs(i);

                if isempty(traj.hypotheses) || trajExistProb < obj.Config.existenceThreshold
                    continue;
                end

                % Find hypothesis with maximum existence probability
                maxExistProb = 0;
                bestHypIdx = 1;
                for h = 1:length(traj.hypotheses)
                    if ~isempty(traj.hypotheses{h}) && traj.hypotheses{h}.existProb > maxExistProb
                        maxExistProb = traj.hypotheses{h}.existProb;
                        bestHypIdx = h;
                    end
                end

                % Extract the current state (last time step)
                bestHyp = traj.hypotheses{bestHypIdx};
                if ~isempty(bestHyp.states)
                    numSteps = size(bestHyp.states, 2);
                    currentState = bestHyp.states(:, numSteps);
                    states = [states, currentState];
                    weights = [weights, trajExistProb];
                end
            end

            estimates.states = states;
            estimates.weights = weights;
            estimates.cardinality = size(states, 2);
        end
        
        function obj = prune(obj)
            obj.PoissonComponent = obj.PoissonComponent.prune(obj.Config.pruningThreshold);
            obj = obj.pruneTrajectoryComponents();
        end
    end
    
    methods (Access = private)
        function obj = updateTrajectoryComponents(obj, z, H, R, pD, lambdaC)
            numMeasurements = size(z, 2);
            numTrajs = length(obj.TrajectoryComponents.existProbs);
            
            if numTrajs == 0
                return;
            end
            
            for i = 1:numTrajs
                traj = obj.TrajectoryComponents.trajectories{i};
                numHyps = length(traj.hypotheses);
                
                newHypotheses = cell(numHyps + numMeasurements, 1);
                
                r_i = obj.TrajectoryComponents.existProbs(i);
                
                for h = 1:numHyps
                    newHypotheses{h} = traj.hypotheses{h};
                    newHypotheses{h}.detectionHistory = [traj.hypotheses{h}.detectionHistory, 0];
                    newHypotheses{h}.existProb = r_i * (1 - pD);
                end
                
                for m = 1:numMeasurements
                    zM = z(:, m);
                    
                    for h = 1:numHyps
                        hyp = traj.hypotheses{h};
                        numSteps = size(hyp.states, 2);
                        
                        xPred = hyp.states(:, numSteps);
                        
                        if length(hyp.covariances) >= numSteps && ~isempty(hyp.covariances{numSteps})
                            PPred = hyp.covariances{numSteps};
                        else
                            PPred = obj.Config.motionModel.Q;
                        end
                        
                        S = H * PPred * H' + R;
                        S = (S + S') / 2;
                        K = PPred * H' / S;
                        zInnov = zM - H * xPred(:);
                        
                        xUpdated = xPred + K * zInnov;
                        PUpdated = PPred - K * S * K';
                        
                        lik = obj.computeLikelihood(zInnov, S);
                        
                        newHyp = struct();
                        newHyp.states = [hyp.states, xUpdated];
                        newHyp.covariances = hyp.covariances;
                        newHyp.covariances{numSteps + 1} = PUpdated;
                        newHyp.detectionHistory = [hyp.detectionHistory, m];
                        
                        denom = lambdaC + r_i * pD * lik;
                        if denom > eps
                            newHyp.existProb = r_i * pD * lik / denom;
                        else
                            newHyp.existProb = 0;
                        end
                        
                        newHypotheses{numHyps + m} = newHyp;
                    end
                end
                
                traj.hypotheses = newHypotheses;
                obj.TrajectoryComponents.trajectories{i} = traj;
                
                totalExist = 0;
                for h = 1:length(newHypotheses)
                    if ~isempty(newHypotheses{h})
                        totalExist = totalExist + newHypotheses{h}.existProb;
                    end
                end
                obj.TrajectoryComponents.existProbs(i) = min(totalExist, 1);
            end
        end
        
        function obj = projectToTPMB(obj)
            numTrajs = length(obj.TrajectoryComponents.existProbs);
            
            for i = 1:numTrajs
                traj = obj.TrajectoryComponents.trajectories{i};
                numHyps = length(traj.hypotheses);
                
                if numHyps <= 1
                    continue;
                end
                
                existProbs = zeros(numHyps, 1);
                for h = 1:numHyps
                    if ~isempty(traj.hypotheses{h})
                        existProbs(h) = traj.hypotheses{h}.existProb;
                    end
                end
                
                totalExist = sum(existProbs);
                if totalExist > obj.ProjectionThreshold
                    weights = existProbs / totalExist;
                    
                    maxSteps = 0;
                    for h = 1:numHyps
                        if ~isempty(traj.hypotheses{h})
                            maxSteps = max(maxSteps, size(traj.hypotheses{h}.states, 2));
                        end
                    end
                    
                    stateDim = size(obj.Config.motionModel.F, 1);
                    mergedStates = zeros(stateDim, maxSteps);
                    mergedCovs = cell(1, maxSteps);
                    mergedDetHistory = zeros(1, maxSteps);
                    
                    for k = 1:maxSteps
                        stateSum = zeros(stateDim, 1);
                        covSum = zeros(stateDim, stateDim);
                        weightSum = 0;
                        
                        for h = 1:numHyps
                            if ~isempty(traj.hypotheses{h}) && size(traj.hypotheses{h}.states, 2) >= k
                                h_state = traj.hypotheses{h}.states(:, k);
                                h_state = h_state(:);  % 确保是列向量
                                stateSum = stateSum + weights(h) * h_state;
                                if length(traj.hypotheses{h}.covariances) >= k && ~isempty(traj.hypotheses{h}.covariances{k})
                                    h_cov = traj.hypotheses{h}.covariances{k};
                                    covSum = covSum + weights(h) * h_cov;
                                end
                                weightSum = weightSum + weights(h);
                            end
                        end
                        
                        if weightSum > 0
                            mergedStates(:, k) = stateSum / weightSum;
                            mergedCovs{k} = covSum / weightSum;
                            mergedDetHistory(k) = 1;
                        end
                    end
                    
                    mergedHyp = struct();
                    mergedHyp.states = mergedStates;
                    mergedHyp.covariances = mergedCovs;
                    mergedHyp.detectionHistory = mergedDetHistory;
                    mergedHyp.existProb = totalExist;
                    
                    traj.hypotheses = {mergedHyp};
                    obj.TrajectoryComponents.trajectories{i} = traj;
                    obj.TrajectoryComponents.existProbs(i) = totalExist;
                end
            end
        end
        
        function obj = pruneTrajectoryComponents(obj)
            existThreshold = obj.Config.existenceThreshold;
            keepIdx = obj.TrajectoryComponents.existProbs >= existThreshold;
            
            obj.TrajectoryComponents.trajectories = obj.TrajectoryComponents.trajectories(keepIdx);
            obj.TrajectoryComponents.existProbs = obj.TrajectoryComponents.existProbs(keepIdx);
            obj.TrajectoryComponents.trackIds = obj.TrajectoryComponents.trackIds(keepIdx);
            obj.TrajectoryComponents.birthTimes = obj.TrajectoryComponents.birthTimes(keepIdx);
            obj.TrajectoryComponents.deathTimes = obj.TrajectoryComponents.deathTimes(keepIdx);
        end
        
        function obj = createTrajectoriesFromPoisson(obj, z, H, R, pD, lambdaC)
            % CREATETRAJECTORIESFROMPOISSON 从泊松分量创建新轨迹
            %
            % 修复：添加门控检查，只有通过门控的量测才创建新轨迹

            numMeasurements = size(z, 2);

            for m = 1:numMeasurements
                zM = z(:, m);

                poissonIntensity = obj.PoissonComponent.getIntensity();

                if poissonIntensity < eps
                    continue;
                end

                % 获取birthModel用于门控检查
                birthModel = obj.Config.getBirthModel('single');

                % 计算与birthModel的马氏距离（门控检查）
                S_birth = H * birthModel.cov * H' + R;
                zPred_birth = H * birthModel.mean;
                innov_birth = zM - zPred_birth;
                maha_birth = innov_birth' * (S_birth \ innov_birth);

                % 门控检查：只有通过门控的量测才创建新轨迹
                if maha_birth > obj.Config.gatingThreshold
                    continue;  % 跳过未通过门控的量测（很可能是杂波）
                end

                % 通过门控，创建新轨迹
                K = birthModel.cov * H' / S_birth;
                birthMean = birthModel.mean + K * innov_birth;
                birthCov = birthModel.cov - K * S_birth * K';

                newTrackId = max([0, obj.TrajectoryComponents.trackIds]) + 1;

                newHyp = struct();
                newHyp.states = birthMean;
                newHyp.covariances = {birthCov};
                newHyp.detectionHistory = m;
                newHyp.existProb = birthModel.existProb;

                newTraj = struct();
                newTraj.hypotheses = {newHyp};

                obj.TrajectoryComponents.trajectories{end+1} = newTraj;
                obj.TrajectoryComponents.existProbs(end+1) = birthModel.existProb;
                obj.TrajectoryComponents.trackIds(end+1) = newTrackId;
                obj.TrajectoryComponents.birthTimes(end+1) = obj.CurrentTime;
                obj.TrajectoryComponents.deathTimes(end+1) = obj.CurrentTime;
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
            
            numTrajs = length(obj.TrajectoryComponents.trajectories);
            
            if numTrajs == 0
                zGated = z;
                indices = 1:size(z, 2);
                distances = zeros(1, size(z, 2));
                return;
            end
            
            keepIdx = true(1, size(z, 2));
            
            for m = 1:size(z, 2)
                zM = z(:, m);
                minDist = inf;
                
                for i = 1:length(obj.TrajectoryComponents.trajectories)
                    traj = obj.TrajectoryComponents.trajectories{i};
                    for h = 1:length(traj.hypotheses)
                        if ~isempty(traj.hypotheses{h})
                            hyp = traj.hypotheses{h};
                            numSteps = size(hyp.states, 2);
                            xPred = hyp.states(:, numSteps);
                            
                            if iscell(hyp.covariances) && length(hyp.covariances) >= numSteps
                                PPred = hyp.covariances{numSteps};
                            else
                                PPred = obj.Config.motionModel.Q;
                            end
                            
                            S = H * PPred * H' + R;
                            zInnov = zM - H * xPred;
                            dist = zInnov' * (S \ zInnov);
                            
                            if dist < minDist
                                minDist = dist;
                            end
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
