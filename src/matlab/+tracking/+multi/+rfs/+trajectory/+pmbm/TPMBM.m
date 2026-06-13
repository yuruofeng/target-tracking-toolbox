classdef TPMBM < tracking.multi.rfs.core.BaseFilter
    % TPMBM 轨迹泊松多伯努利混合滤波器
    %
    % 实现TPMBM滤波器用于多目标跟踪
    %
    % 参考文献:
    %   K. Granström, L. Svensson, Y. Xia, J. Williams, Á. F. García-Fernández,
    %   "Poisson multi-Bernoulli mixture trackers: Continuity through random 
    %   finite sets of trajectories," FUSION 2018
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('motionModel', struct('type', 'CV', 'F', F, 'Q', Q), ...
    %                               'measurementModel', struct('type', 'Linear', 'H', H, 'R', R), ...
    %                               'pruningThreshold', 1e-4, ...
    %                               'maxComponents', 1000, ...
    %                               'gateSize', 9.210);
    %   filter = tracking.multi.rfs.trajectory.pmbm.TPMBM(config);
    %   result = filter.run(measurements, groundTruth);
    %

    properties (Access = private)
        % 滤波器状态
        TrajectoryPPP     % 轨迹PPP分量
        TrajectoryBernoulli % 轨迹Bernoulli分量
        NewlyDetectedTrajectory % 新检测的轨迹
        MeasurementAssociatedToNew % 与新轨迹关联的测量
        GateSize         % 门控大小
        MaxGlobalHypotheses % 最大全局假设数
        MinGlobalHypothesisWeight % 最小全局假设权重
        NumMinimumAssignment % 最小分配数
        MinEndTimeProbability % 最小结束时间概率
        MinBirthTimeProbability % 最小出生时间概率
        MinExistenceProbability % 最小存在概率
    end
    
    methods
        function obj = TPMBM(config)
            % TPMBM 构造函数
            %
            % 输入:
            %   config - FilterConfig对象
            
            % 调用基类构造函数
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            
            % 初始化TPMBM特定参数
            obj.GateSize = config.gatingThreshold;
            obj.MaxGlobalHypotheses = 100;
            obj.MinGlobalHypothesisWeight = 1e-5;
            obj.NumMinimumAssignment = 3;
            obj.MinEndTimeProbability = 0.1;
            obj.MinBirthTimeProbability = 0.5;
            obj.MinExistenceProbability = config.existenceThreshold;
            
            % 初始化状态
            obj.TrajectoryPPP = [];
            obj.TrajectoryBernoulli = {};
            obj.NewlyDetectedTrajectory = {};
            obj.MeasurementAssociatedToNew = {};
        end
        
        function obj = initialize(obj)
            % INITIALIZE 初始化滤波器
            
            % 重置状态
            obj.TrajectoryPPP = [];
            obj.TrajectoryBernoulli = {};
            obj.NewlyDetectedTrajectory = {};
            obj.MeasurementAssociatedToNew = {};
            obj.CurrentTime = 0;
        end
        
        function obj = predict(obj)
            % PREDICT 预测步骤
            
            if obj.CurrentTime == 0
                return;
            end
            
            % 预测未检测轨迹（PPP分量）
            obj.predictPPPTrajectories();
            obj = obj.predictPPPTrajectories();
            
            % 预测已检测轨迹（Bernoulli分量）
            obj.predictBernoulliTrajectories();
            obj = obj.predictBernoulliTrajectories();
        end
        
        function obj = update(obj, measurement)
            if obj.CurrentTime < 1
                obj.CurrentTime = 1;
            end
            % UPDATE 更新步骤
            %
            % 输入:
            %   measurement - 当前时刻的测量
            
            % 更新已检测轨迹
            obj.updateBernoulliTrajectories(measurement);
            obj = obj.updateBernoulliTrajectories(measurement);
            
            % 更新未检测轨迹并处理新检测
            obj.updatePPPTrajectories(measurement);
            obj = obj.updatePPPTrajectories(measurement);
            
            % 执行数据关联
            obj = obj.performDataAssociation(measurement);
        end
        
        function estimate = estimate(obj)
            % ESTIMATE 估计步骤
            %
            % 输出:
            %   estimate - 估计结果
            
            % 收集所有活跃轨迹的状态
            states = [];
            weights = [];
            
            % 从Bernoulli分量中提取轨迹
            for i = 1:length(obj.TrajectoryBernoulli)
                bernoulli = obj.TrajectoryBernoulli{i};
                for j = 1:length(bernoulli)
                    hyp = bernoulli(j);
                    if hyp.isAlive && hyp.existenceProbability > obj.MinExistenceProbability
                        for k = 1:length(hyp.trajectoryMixture)
                            traj = hyp.trajectoryMixture(k);
                            states = [states, traj.marginalMean];
                            weights = [weights, hyp.existenceProbability * hyp.birthTimeProbability(k)];
                        end
                    end
                end
            end
            
            % 计算 cardinality 估计
            cardinality = sum(weights);
            
            estimate = struct('states', states, 'weights', weights, 'cardinality', cardinality);
        end
        
        function obj = prune(obj)
            % PRUNE 剪枝步骤
            
            % 剪枝PPP分量
            obj.prunePPPTrajectories();
            
            % 剪枝Bernoulli分量
            obj.pruneBernoulliTrajectories();
        end
    end
    
    methods (Access = protected)
        function obj = predictPPPTrajectories(obj)
            % PREDICTP 预测PPP轨迹
            
            F = obj.Config.motionModel.F;
            Q = obj.Config.motionModel.Q;
            p_s = obj.Config.survivalProb;
            
            for i = 1:length(obj.TrajectoryPPP)
                % 更新存活概率
                obj.TrajectoryPPP(i).weight = obj.TrajectoryPPP(i).weight * p_s;
                
                % 状态预测
                obj.TrajectoryPPP(i).marginalMean = F * obj.TrajectoryPPP(i).marginalMean;
                
                % 协方差预测
                obj.TrajectoryPPP(i).marginalCovariance = F * obj.TrajectoryPPP(i).marginalCovariance * F' + Q;
                
                % 更新结束时间
                obj.TrajectoryPPP(i).endTime = obj.TrajectoryPPP(i).endTime + 1;
            end
        end
        
        function obj = predictBernoulliTrajectories(obj)
            % PREDICTBERNOULLI 预测Bernoulli轨迹
            
            F = obj.Config.motionModel.F;
            Q = obj.Config.motionModel.Q;
            p_s = obj.Config.survivalProb;
            
            if obj.CurrentTime <= 1
                return;
            end
            
            for i = 1:length(obj.TrajectoryBernoulli)
                bernoulli = obj.TrajectoryBernoulli{i};
                for j = 1:length(bernoulli)
                    hyp = bernoulli(j);
                    if hyp.isAlive
                        for k = 1:length(hyp.trajectoryMixture)
                            % 状态预测
                            hyp.trajectoryMixture(k).marginalMean = F * hyp.trajectoryMixture(k).marginalMean;
                            
                            % 协方差预测
                            hyp.trajectoryMixture(k).marginalCovariance = F * hyp.trajectoryMixture(k).marginalCovariance * F' + Q;
                            
                            % 更新结束时间
                            hyp.endTime(k) = hyp.endTime(k) + 1;
                            
                            % 更新结束时间概率（索引保护）
                            if obj.CurrentTime > 1 && size(hyp.endTimeProbability, 2) >= obj.CurrentTime - 1
                                hyp.endTimeProbability(k, obj.CurrentTime) = hyp.endTimeProbability(k, obj.CurrentTime - 1) * p_s;
                                hyp.endTimeProbability(k, obj.CurrentTime - 1) = hyp.endTimeProbability(k, obj.CurrentTime - 1) * (1 - p_s);
                            end
                        end
                        bernoulli(j) = hyp;
                    end
                end
                obj.TrajectoryBernoulli{i} = bernoulli;
            end
        end
        
        function obj = updateBernoulliTrajectories(obj, measurement)
            % UPDATEBERNOULLI 更新Bernoulli轨迹
            
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            p_d = obj.Config.detectionProb;
            
            numMeasurements = size(measurement, 2);
            updatedBernoulli = cell(1, length(obj.TrajectoryBernoulli));
            
            for i = 1:length(obj.TrajectoryBernoulli)
                bernoulli = obj.TrajectoryBernoulli{i};
                updatedBernoulli{i} = [];
                
                for j = 1:length(bernoulli)
                    hyp = bernoulli(j);
                    
                    % 处理漏检假设
                    misDetectionHyp = obj.processMisDetectionHypothesis(hyp, measurement);
                    updatedBernoulli{i} = [updatedBernoulli{i}, misDetectionHyp];
                    
                    % 处理测量更新假设
                    for m = 1:numMeasurements
                        measurementHyp = obj.processMeasurementUpdateHypothesis(hyp, measurement(:, m), m);
                        if ~isempty(measurementHyp)
                            updatedBernoulli{i} = [updatedBernoulli{i}, measurementHyp];
                        end
                    end
                end
            end
            
            obj.TrajectoryBernoulli = updatedBernoulli;
        end
        
        function obj = updatePPPTrajectories(obj, measurement)
            % UPDATEPP 更新PPP轨迹
            
            % 添加新生轨迹
            obj.addBirthTrajectories();
            
            % 处理新检测
            obj.processNewTrajectories(measurement);
        end
        
        function obj = addBirthTrajectories(obj)
            % ADDBIRTH 添加新生轨迹
            
            if isfield(obj.Config.extraParams, 'birthComponents')
                birthComponents = obj.Config.extraParams.birthComponents;
                for i = 1:length(birthComponents)
                    newTraj = struct();
                    newTraj.weight = birthComponents(i).weight;
                    newTraj.marginalMean = birthComponents(i).mean;
                    newTraj.marginalCovariance = birthComponents(i).covariance;
                    newTraj.birthTime = obj.CurrentTime;
                    newTraj.endTime = obj.CurrentTime;
                    obj.TrajectoryPPP = [obj.TrajectoryPPP, newTraj];
                end
            end
        end
        
        function obj = processNewTrajectories(obj, measurement)
            % PROCESSNEW 处理新轨迹
            
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            p_d = obj.Config.detectionProb;
            
            numMeasurements = size(measurement, 2);
            obj.NewlyDetectedTrajectory{obj.CurrentTime} = cell(1, numMeasurements);
            obj.MeasurementAssociatedToNew{obj.CurrentTime} = false(1, numMeasurements);
            
            for m = 1:numMeasurements
                meas = measurement(:, m);
                associated = false;
                
                % 检查与PPP轨迹的关联
                for i = 1:length(obj.TrajectoryPPP)
                    traj = obj.TrajectoryPPP(i);
                    
                    % 计算预测测量
                    zPred = H * traj.marginalMean;
                    S = H * traj.marginalCovariance * H' + R;
                    
                    % 计算马氏距离
                    innovation = meas - zPred;
                    mahalanobis = innovation' * inv(S) * innovation;
                    
                    if mahalanobis < obj.GateSize
                        % 创建新的Bernoulli轨迹
                        newHyp = struct();
                        newHyp.trajectoryMixture = traj;
                        newHyp.birthTime = traj.birthTime;
                        newHyp.birthTimeProbability = 1;
                        newHyp.endTime = traj.endTime;
                        newHyp.endTimeProbability = zeros(1, obj.Config.extraParams.totalTimeSteps);
                        newHyp.endTimeProbability(obj.CurrentTime) = 1;
                        newHyp.existenceProbability = 1;
                        newHyp.isAlive = true;
                        newHyp.associationHistory = zeros(1, obj.Config.extraParams.totalTimeSteps);
                        newHyp.associationHistory(obj.CurrentTime) = m;
                        
                        obj.NewlyDetectedTrajectory{obj.CurrentTime}{m} = newHyp;
                        obj.MeasurementAssociatedToNew{obj.CurrentTime}(m) = true;
                        associated = true;
                        break;
                    end
                end
            end
        end
        
        function obj = performDataAssociation(obj, measurement)
            % PERFORMDATAASSOCIATION 执行数据关联
            
            % 这里实现简化的数据关联逻辑
            % 实际应用中应该使用更复杂的K最佳分配算法
            
            numMeasurements = size(measurement, 2);
            
            % 添加新检测的轨迹
            hasNewTrajectories = obj.CurrentTime > 0 && ...
                numel(obj.NewlyDetectedTrajectory) >= obj.CurrentTime && ...
                ~isempty(obj.NewlyDetectedTrajectory{obj.CurrentTime});

            if hasNewTrajectories
                newTrajectories = obj.NewlyDetectedTrajectory{obj.CurrentTime};
                for m = 1:numMeasurements
                    hasAssociationFlag = numel(obj.MeasurementAssociatedToNew) >= obj.CurrentTime && ...
                        numel(obj.MeasurementAssociatedToNew{obj.CurrentTime}) >= m && ...
                        obj.MeasurementAssociatedToNew{obj.CurrentTime}(m);

                    if hasAssociationFlag
                        if ~isempty(newTrajectories{m})
                            obj.TrajectoryBernoulli{end+1} = newTrajectories{m};
                        end
                    end
                end
            end
        end
        
        function obj = prunePPPTrajectories(obj)
            % PRUNEPPP 剪枝PPP轨迹
            
            % 移除权重低于阈值的PPP分量
            keepIndices = [];
            for i = 1:length(obj.TrajectoryPPP)
                if obj.TrajectoryPPP(i).weight > obj.Config.pruningThreshold
                    keepIndices = [keepIndices, i];
                end
            end
            
            if ~isempty(keepIndices)
                obj.TrajectoryPPP = obj.TrajectoryPPP(keepIndices);
            else
                obj.TrajectoryPPP = [];
            end
        end
        
        function obj = pruneBernoulliTrajectories(obj)
            % PRUNBERNOULLI 剪枝Bernoulli轨迹
            
            updatedBernoulli = {};
            for i = 1:length(obj.TrajectoryBernoulli)
                bernoulli = obj.TrajectoryBernoulli{i};
                keepHypotheses = [];
                
                for j = 1:length(bernoulli)
                    hyp = bernoulli(j);
                    if hyp.existenceProbability > obj.MinExistenceProbability
                        % 剪枝轨迹混合分量
                        keepMixtures = [];
                        for k = 1:length(hyp.trajectoryMixture)
                            if hyp.birthTimeProbability(k) > obj.MinBirthTimeProbability
                                keepMixtures = [keepMixtures, k];
                            end
                        end
                        
                        if ~isempty(keepMixtures)
                            hyp.trajectoryMixture = hyp.trajectoryMixture(keepMixtures);
                            hyp.birthTimeProbability = hyp.birthTimeProbability(keepMixtures);
                            hyp.birthTimeProbability = hyp.birthTimeProbability / sum(hyp.birthTimeProbability);
                            hyp.birthTime = hyp.birthTime(keepMixtures);
                            hyp.endTime = hyp.endTime(keepMixtures);
                            hyp.endTimeProbability = hyp.endTimeProbability(keepMixtures, :);
                            keepHypotheses = [keepHypotheses, hyp];
                        end
                    end
                end
                
                if ~isempty(keepHypotheses)
                    updatedBernoulli{end+1} = keepHypotheses;
                end
            end
            
            obj.TrajectoryBernoulli = updatedBernoulli;
        end
        
        function hyp = processMisDetectionHypothesis(obj, hyp, measurement)
            % PROCESSMISDETECTION 处理漏检假设
            
            p_d = obj.Config.detectionProb;
            hyp.existenceProbability = hyp.existenceProbability * (1 - p_d);
            hyp.isAlive = true;
            
            % 更新轨迹混合分量
            for k = 1:length(hyp.trajectoryMixture)
                hyp.endTimeProbability(k, obj.CurrentTime) = hyp.endTimeProbability(k, obj.CurrentTime) * (1 - p_d);
            end
            
            return;
        end
        
        function hyp = processMeasurementUpdateHypothesis(obj, hyp, measurement, measIdx)
            % PROCESSMEASUREMENTUPDATE 处理测量更新假设
            
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            p_d = obj.Config.detectionProb;
            
            % 计算预测测量
            zPred = H * hyp.trajectoryMixture.marginalMean;
            S = H * hyp.trajectoryMixture.marginalCovariance * H' + R;
            
            % 计算马氏距离
            innovation = measurement - zPred;
            mahalanobis = innovation' * inv(S) * innovation;
            
            if mahalanobis < obj.GateSize
                % 计算卡尔曼增益
                K = hyp.trajectoryMixture.marginalCovariance * H' * inv(S);
                
                % 更新状态
                hyp.trajectoryMixture.marginalMean = hyp.trajectoryMixture.marginalMean + K * innovation;
                hyp.trajectoryMixture.marginalCovariance = (eye(size(K, 1)) - K * H) * hyp.trajectoryMixture.marginalCovariance;
                
                % 更新存在概率
                hyp.existenceProbability = 1;
                
                % 更新关联历史
                hyp.associationHistory(obj.CurrentTime) = measIdx;
                
                return;
            else
                hyp = [];
                return;
            end
        end
    end
end
