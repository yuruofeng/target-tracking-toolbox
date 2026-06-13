classdef MBMComponent
    % MBMCOMPONENT 多伯努利混合分量
    %
    % 实现PMBM滤波器中的多伯努利混合分量
    %
    % 使用示例:
    %   mbm = tracking.multi.rfs.pmbm.MBMComponent(config);
    %   mbm = mbm.predict();
    %   mbm = mbm.update(z, H, R);
    %

    properties
        Tracks         cell   = {}  % 伯努利分量列表
        GlobalHyp      double = []  % 全局假设矩阵
        GlobalHypWeight double = []  % 全局假设权重
    end
    
    properties (Access = private)
        Config
        NextTrackId = 1  % 下一个航迹ID
    end
    
    methods
        function obj = MBMComponent(config)
            % MBMCOMPONENT 构造函数
            
            if nargin < 1
                ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                    '必须提供config参数');
                throw(ME);
            end
            
            obj.Config = config;
        end
        
        function obj = initialize(obj)
            % INITIALIZE 初始化MBM分量
            
            obj.Tracks = {};
            obj.GlobalHyp = [];
            obj.GlobalHypWeight = [];
        end
        
        function obj = predict(obj)
            % PREDICT 预测MBM分量
            
            pS = obj.Config.survivalProb;
            F = obj.Config.motionModel.F;
            Q = obj.Config.motionModel.Q;
            
            % 预测每个航迹
            for i = 1:length(obj.Tracks)
                track = obj.Tracks{i};
                
                % 预测状态
                track.state = F * track.state;
                track.covariance = F * track.covariance * F' + Q;
                track.covariance = (track.covariance + track.covariance') / 2;
                
                % 预测存在概率
                track.existenceProb = pS * track.existenceProb;
                
                obj.Tracks{i} = track;
            end
        end
        
        function obj = update(obj, z, H, R, pD, lambdaC, existenceThreshold)
            % UPDATE 更新MBM分量

            if isempty(z)
                return;
            end

            numMeas = size(z, 2);

            % 对每个量测创建新的潜在航迹
            for j = 1:numMeas
                z_j = z(:, j);
                obj = obj.createTrackFromMeasurement(z_j, H, R, pD, lambdaC);
            end

            % 更新现有航迹（漏检更新）
            for i = 1:length(obj.Tracks)
                track = obj.Tracks{i};
                track.existenceProb = (1 - pD) * track.existenceProb;
                obj.Tracks{i} = track;
            end
        end
        
        function obj = prune(obj, weightThreshold, maxHypotheses)
            % PRUNE 剪枝MBM分量
            
            % 1. 移除权重小的全局假设
            if ~isempty(obj.GlobalHypWeight)
                keepIdx = obj.GlobalHypWeight >= weightThreshold;
                obj.GlobalHyp = obj.GlobalHyp(:, keepIdx);
                obj.GlobalHypWeight = obj.GlobalHypWeight(keepIdx);
                
                % 限制最大假设数
                if length(obj.GlobalHypWeight) > maxHypotheses
                    [~, sortedIdx] = sort(obj.GlobalHypWeight, 'descend');
                    keepIdx = sortedIdx(1:maxHypotheses);
                    
                    obj.GlobalHyp = obj.GlobalHyp(:, keepIdx);
                    obj.GlobalHypWeight = obj.GlobalHypWeight(keepIdx);
                end
                
                % 归一化权重（添加除零保护）
                weightSum = sum(obj.GlobalHypWeight);
                if weightSum > eps
                    obj.GlobalHypWeight = obj.GlobalHypWeight / weightSum;
                else
                    numHyps = length(obj.GlobalHypWeight);
                    if numHyps > 0
                        obj.GlobalHypWeight = ones(1, numHyps) / numHyps;
                    end
                end
            end
            
            % 2. 移除未使用的航迹
            usedTrackIndices = unique(obj.GlobalHyp(:));
            usedTrackIndices(usedTrackIndices == 0) = [];

            if ~isempty(usedTrackIndices)
                obj.Tracks = obj.Tracks(usedTrackIndices);

                maxOldIdx = max(usedTrackIndices);
                indexMap = zeros(maxOldIdx, 1);
                for newIdx = 1:length(usedTrackIndices)
                    oldIdx = usedTrackIndices(newIdx);
                    indexMap(oldIdx) = newIdx;
                end

                [numRows, numHyp] = size(obj.GlobalHyp);
                for r = 1:numRows
                    for c = 1:numHyp
                        oldIdx = obj.GlobalHyp(r, c);
                        if oldIdx > 0 && oldIdx <= maxOldIdx
                            obj.GlobalHyp(r, c) = indexMap(oldIdx);
                        end
                    end
                end

                obj.GlobalHyp = obj.GlobalHyp(1:length(usedTrackIndices), :);
            else
                obj.Tracks = {};
                obj.GlobalHyp = [];
                obj.GlobalHypWeight = [];
            end
        end
        
        function estimate = estimate(obj, method)
            % ESTIMATE 估计目标状态
            
            if nargin < 2
                method = 1;  % 默认使用方法1
            end
            
            estimate = struct('states', [], 'existenceProbs', []);
            
            if isempty(obj.GlobalHypWeight)
                return;
            end
            
            % 选择权重最大的全局假设
            [~, maxIdx] = max(obj.GlobalHypWeight);
            selectedHyp = obj.GlobalHyp(:, maxIdx);
            
            % 提取该假设中的航迹
            trackIndices = unique(selectedHyp(selectedHyp > 0));
            
            states = [];
            existenceProbs = [];
            
            for i = 1:length(trackIndices)
                trackIdx = trackIndices(i);
                if trackIdx <= length(obj.Tracks)
                    track = obj.Tracks{trackIdx};
                    states = [states, track.state];
                    existenceProbs = [existenceProbs, track.existenceProb];
                end
            end
            
            estimate.states = states;
            estimate.existenceProbs = existenceProbs;
        end

        function obj = createTrackFromMeasurement(obj, z, H, R, pD, lambdaC)
            % CREATETRACKFROMMEASUREMENT 从量测创建新航迹

            track = struct();

            stateDim = size(H, 2);
            measDim = size(H, 1);

            track.state = zeros(stateDim, 1);
            track.state(1:measDim) = z;

            track.covariance = eye(stateDim) * 100;
            track.covariance(1:measDim, 1:measDim) = R;

            track.existenceProb = 0.8;

            track.id = obj.NextTrackId;
            obj.NextTrackId = obj.NextTrackId + 1;
            track.birthTime = 1;

            trackIdx = length(obj.Tracks) + 1;
            obj.Tracks{trackIdx} = track;

            obj = obj.addGlobalHypothesis(trackIdx);
        end
        
        function obj = addGlobalHypothesis(obj, trackIdx)
            % ADDGLOBALHYPOTHESIS 添加新的全局假设

            numTracks = length(obj.Tracks);

            if isempty(obj.GlobalHyp)
                obj.GlobalHyp = zeros(numTracks, 1);
                obj.GlobalHyp(trackIdx) = 1;
                obj.GlobalHypWeight = 1;
            else
                [currentRows, numExistingHyp] = size(obj.GlobalHyp);
                if currentRows < numTracks
                    obj.GlobalHyp = [obj.GlobalHyp; zeros(numTracks - currentRows, numExistingHyp)];
                end

                newHyp = zeros(numTracks, 1);
                newHyp(trackIdx) = 1;

                obj.GlobalHyp = [obj.GlobalHyp, newHyp];
                obj.GlobalHypWeight = [obj.GlobalHypWeight, 1];

                weightSum = sum(obj.GlobalHypWeight);
                if weightSum > eps
                    obj.GlobalHypWeight = obj.GlobalHypWeight / weightSum;
                else
                    numHyps = length(obj.GlobalHypWeight);
                    if numHyps > 0
                        obj.GlobalHypWeight = ones(1, numHyps) / numHyps;
                    end
                end
            end
        end
    end
end
