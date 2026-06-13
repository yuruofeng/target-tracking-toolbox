classdef TrajectoryErrorCalculator < handle
    % TRAJECTORYERRORCALCULATOR 轨迹误差计算工具
    %
    % 实现GOSPA和LP轨迹度量误差计算
    %
    % 参考文献:
    %   A. S. Rahmathullah, A. F. Garcia-Fernandez and L. Svensson, 
    %   "Trajectory PHD and CPHD filters", 
    %   IEEE Trans. Signal Processing, vol. 67, no. 22, pp. 5702-5714, Nov. 2019.
    %
    % 使用示例:
    %   calc = tracking.metrics.TrajectoryErrorCalculator('c', 10, 'p', 2);
    %   [squaredGOSPA, loc, mis, fal] = calc.computeGOSPAErrorTrajectory(...
    %       X_estimate, t_b_estimate, length_estimate, ...
    %       X_truth, t_birth, t_death, k_end);
    %

    properties
        p       (1,1) double = 2
        c       (1,1) double = 10
        alpha   (1,1) double = 2
        gamma   (1,1) double = 1
        Nx      (1,1) double = 4
        Results = struct()
    end
    
    properties (Access = private)
        GOSPAInstance
    end
    
    methods
        function obj = TrajectoryErrorCalculator(varargin)
            for i = 1:2:length(varargin)
                if i+1 > length(varargin)
                    break;
                end
                key = varargin{i};
                value = varargin{i+1};
                
                switch lower(key)
                    case 'c'
                        obj.c = value;
                    case 'p'
                        obj.p = value;
                    case 'alpha'
                        obj.alpha = value;
                    case 'gamma'
                        obj.gamma = value;
                    case 'nx'
                        obj.Nx = value;
                end
            end
            
            obj.GOSPAInstance = tracking.metrics.GOSPA('p', obj.p, 'c', obj.c, 'alpha', obj.alpha);
        end
        
        function [squaredGOSPA, loc, mis, fal] = computeGOSPAErrorTrajectory(obj, ...
                X_estimate, t_b_estimate, length_estimate, ...
                X_truth, t_birth, t_death, k_end)
            
            Nx = obj.Nx;
            numEstimates = length(X_estimate);
            numTargets = length(t_birth);
            
            squaredGOSPA = 0;
            loc = 0;
            mis = 0;
            fal = 0;
            
            for k = 1:k_end
                aliveTargetsIndex = (k >= t_birth) & (k < t_death);
                nAliveTargets = sum(aliveTargetsIndex);
                
                if nAliveTargets > 0
                    Xk_pos = zeros(2, nAliveTargets);
                    idx = 1;
                    for i = 1:numTargets
                        if aliveTargetsIndex(i)
                            Xk_pos(1, idx) = X_truth((i-1)*Nx + 1, k);
                            Xk_pos(2, idx) = X_truth((i-1)*Nx + 3, k);
                            idx = idx + 1;
                        end
                    end
                else
                    Xk_pos = zeros(2, 0);
                end
                
                if numEstimates > 0
                    aliveEstimateIndex = find((k >= t_b_estimate) & ...
                        (k <= t_b_estimate + length_estimate - 1));
                    nAliveEstimates = length(aliveEstimateIndex);
                else
                    aliveEstimateIndex = [];
                    nAliveEstimates = 0;
                end
                
                if nAliveEstimates > 0
                    Yk = zeros(Nx, nAliveEstimates);
                    for i = 1:nAliveEstimates
                        estIdx = aliveEstimateIndex(i);
                        indexLoc = k - t_b_estimate(estIdx) + 1;
                        if indexLoc > 0 && indexLoc <= length_estimate(estIdx)
                            Yk(:, i) = X_estimate{estIdx}((indexLoc-1)*Nx + 1 : indexLoc*Nx);
                        end
                    end
                    
                    validIdx = ~isnan(Yk(1, :));
                    YkPos = Yk([1, 3], validIdx);
                else
                    YkPos = zeros(2, 0);
                end
                
                [~, ~, decompCost] = obj.GOSPAInstance.compute(Xk_pos, YkPos);
                dGospa = obj.GOSPAInstance.Results.distance;
                
                squaredGOSPA = squaredGOSPA + dGospa^2;
                if isfield(decompCost, 'localisation')
                    loc = loc + decompCost.localisation;
                end
                if isfield(decompCost, 'missed')
                    mis = mis + decompCost.missed;
                end
                if isfield(decompCost, 'false')
                    fal = fal + decompCost.false;
                end
            end
            
            squaredGOSPA = squaredGOSPA / k_end;
            loc = loc / k_end;
            mis = mis / k_end;
            fal = fal / k_end;
            
            obj.Results = struct(...
                'squaredGOSPA', squaredGOSPA, ...
                'localisation', loc, ...
                'missed', mis, ...
                'false', fal, ...
                'kEnd', k_end, ...
                'parameters', struct('p', obj.p, 'c', obj.c, 'alpha', obj.alpha, 'gamma', obj.gamma) ...
            );
        end
        
        function [squaredLP, loc, mis, switchCost] = computeLPMetricError(obj, ...
                X_estimate, t_b_estimate, length_estimate, ...
                X_truth, t_birth, t_death, k_end)
            
            Nx = obj.Nx;
            gamma = obj.gamma;
            
            squaredLP = 0;
            loc = 0;
            mis = 0;
            switchCost = 0;
            
            previousAssignment = [];
            
            for k = 1:k_end
                aliveTargetsIndex = (k >= t_birth) & (k < t_death);
                nAliveTargets = sum(aliveTargetsIndex);
                
                if nAliveTargets > 0
                    Xk_pos = zeros(2, nAliveTargets);
                    idx = 1;
                    for i = 1:length(t_birth)
                        if aliveTargetsIndex(i)
                            Xk_pos(1, idx) = X_truth((i-1)*Nx + 1, k);
                            Xk_pos(2, idx) = X_truth((i-1)*Nx + 3, k);
                            idx = idx + 1;
                        end
                    end
                else
                    Xk_pos = zeros(2, 0);
                end
                
                numEstimates = length(X_estimate);
                if numEstimates > 0
                    aliveEstimateIndex = find((k >= t_b_estimate) & ...
                        (k <= t_b_estimate + length_estimate - 1));
                    nAliveEstimates = length(aliveEstimateIndex);
                else
                    aliveEstimateIndex = [];
                    nAliveEstimates = 0;
                end
                
                if nAliveEstimates > 0
                    Yk = zeros(Nx, nAliveEstimates);
                    for i = 1:nAliveEstimates
                        estIdx = aliveEstimateIndex(i);
                        indexLoc = k - t_b_estimate(estIdx) + 1;
                        if indexLoc > 0 && indexLoc <= length_estimate(estIdx)
                            Yk(:, i) = X_estimate{estIdx}((indexLoc-1)*Nx + 1 : indexLoc*Nx);
                        end
                    end
                    
                    validIdx = ~isnan(Yk(1, :));
                    YkPos = Yk([1, 3], validIdx);
                else
                    YkPos = zeros(2, 0);
                end
                
                [~, assignment, decompCost] = obj.GOSPAInstance.compute(Xk_pos, YkPos);
                dGospa = obj.GOSPAInstance.Results.distance;
                
                if ~isempty(previousAssignment) && ~isempty(assignment)
                    for j = 1:length(assignment)
                        if assignment(j) ~= 0 && assignment(j) <= length(previousAssignment)
                            if previousAssignment(assignment(j)) ~= j && previousAssignment(assignment(j)) ~= 0
                                switchCost = switchCost + gamma * obj.c^obj.p;
                            end
                        end
                    end
                end
                
                previousAssignment = assignment;
                
                squaredLP = squaredLP + dGospa^2;
                if isfield(decompCost, 'localisation')
                    loc = loc + decompCost.localisation;
                end
                if isfield(decompCost, 'missed')
                    mis = mis + decompCost.missed;
                end
            end
            
            squaredLP = squaredLP / k_end;
            loc = loc / k_end;
            mis = mis / k_end;
            switchCost = switchCost / k_end;
            
            obj.Results.LP = struct(...
                'squaredLP', squaredLP, ...
                'localisation', loc, ...
                'missed', mis, ...
                'switch', switchCost, ...
                'kEnd', k_end ...
            );
        end
        
        function displayResults(obj)
            if isempty(fieldnames(obj.Results))
                fprintf('No results to display\n');
                return;
            end
            
            fprintf('\n===== Trajectory Error Results =====\n');
            
            if isfield(obj.Results, 'squaredGOSPA')
                fprintf('GOSPA Results:\n');
                fprintf('  Squared GOSPA: %.4f\n', obj.Results.squaredGOSPA);
                fprintf('  Localisation: %.4f\n', obj.Results.localisation);
                fprintf('  Missed: %.4f\n', obj.Results.missed);
                fprintf('  False: %.4f\n', obj.Results.false);
            end
            
            if isfield(obj.Results, 'LP')
                fprintf('\nLP Metric Results:\n');
                fprintf('  Squared LP: %.4f\n', obj.Results.LP.squaredLP);
                fprintf('  Localisation: %.4f\n', obj.Results.LP.localisation);
                fprintf('  Missed: %.4f\n', obj.Results.LP.missed);
                fprintf('  Switch: %.4f\n', obj.Results.LP.switch);
            end
            
            fprintf('====================================\n\n');
        end
    end
    
    methods (Static)
        function [squaredGOSPA, loc, mis, fal] = run(X_estimate, t_b_estimate, length_estimate, ...
                X_truth, t_birth, t_death, k_end, varargin)
            calc = tracking.metrics.TrajectoryErrorCalculator(varargin{:});
            [squaredGOSPA, loc, mis, fal] = calc.computeGOSPAErrorTrajectory(...
                X_estimate, t_b_estimate, length_estimate, ...
                X_truth, t_birth, t_death, k_end);
        end
    end
end
