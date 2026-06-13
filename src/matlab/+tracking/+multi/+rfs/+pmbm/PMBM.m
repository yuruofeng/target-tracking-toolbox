classdef PMBM < tracking.multi.rfs.core.BaseFilter
    % PMBM 泊松多伯努利混合滤波器
    %
    % 实现PMBM滤波器，结合泊松分量和多伯努利混合分量
    %
    % 参考文献:
    %   Á. F. García-Fernández, J. L. Williams, K. Granström and L. Svensson, 
    %   "Poisson Multi-Bernoulli Mixture Filter: Direct Derivation and 
    %   Implementation," IEEE Transactions on Aerospace and Electronic Systems, 
    %   vol. 54, no. 4, pp. 1883-1901, Aug. 2018.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('detectionProb', 0.9);
    %   pmbmFilter = tracking.multi.rfs.pmbm.PMBM(config);
    %   result = pmbmFilter.run(measurements, groundTruth);
    %

    properties (Access = private)
        PoissonComponent  % 泊松分量
        MBMComponent      % 多伯努利混合分量
    end
    
    methods
        function obj = PMBM(config)
            % PMBM 构造函数
            
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            
            % 初始化分量
            obj.PoissonComponent = tracking.multi.rfs.pmbm.PoissonComponent(config);
            obj.MBMComponent = tracking.multi.rfs.pmbm.MBMComponent(config);
        end
        
        function obj = initialize(obj)
            % INITIALIZE 初始化滤波器
            
            % 初始化泊松分量
            obj.PoissonComponent = obj.PoissonComponent.initialize();
            
            % 初始化MBM分量
            obj.MBMComponent = obj.MBMComponent.initialize();
            
            % 初始化状态
            obj.State = struct(...
                'poissonIntensity', obj.PoissonComponent.getIntensity(), ...
                'numTracks', length(obj.MBMComponent.Tracks), ...
                'numHypotheses', length(obj.MBMComponent.GlobalHypWeight) ...
            );
        end
        
        function obj = predict(obj)
            % PREDICT 预测步骤
            
            % 预测泊松分量
            obj.PoissonComponent = obj.PoissonComponent.predict();
            
            % 预测MBM分量
            obj.MBMComponent = obj.MBMComponent.predict();
            
            % 更新状态
            obj.State.poissonIntensity = obj.PoissonComponent.getIntensity();
            obj.State.numTracks = length(obj.MBMComponent.Tracks);
            obj.State.numHypotheses = length(obj.MBMComponent.GlobalHypWeight);
        end
        
        function obj = update(obj, measurement)
            % UPDATE 更新步骤

            if isempty(measurement)
                return;
            end

            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            pD = obj.Config.detectionProb;
            lambdaC = obj.Config.clutterRate / prod(obj.Config.surveillanceArea);

            [z_in_gate, ~, ~] = obj.gating(measurement);

            obj.PoissonComponent = obj.PoissonComponent.update(z_in_gate, H, R, pD, lambdaC);
            obj.PoissonComponent = obj.PoissonComponent.prune(obj.Config.pruningThreshold);

            obj.MBMComponent = obj.MBMComponent.update(z_in_gate, H, R, pD, lambdaC, obj.Config.existenceThreshold);

            obj.MBMComponent = obj.MBMComponent.prune(...
                obj.Config.pruningThreshold, ...
                obj.Config.maxComponents);

            obj = obj.createTracksFromPoisson(z_in_gate, H, R, pD, lambdaC);

            obj.State.poissonIntensity = obj.PoissonComponent.getIntensity();
            obj.State.numTracks = length(obj.MBMComponent.Tracks);
            obj.State.numHypotheses = length(obj.MBMComponent.GlobalHypWeight);
        end
        
        function estimate = estimate(obj)
            % ESTIMATE 状态估计
            
            estimate = struct('states', [], 'weights', [], 'cardinality', 0);
            
            % 使用MBM分量的估计
            mbmEst = obj.MBMComponent.estimate(1);  % 使用估计器1
            
            if ~isempty(mbmEst.states)
                estimate.states = mbmEst.states;
                estimate.weights = mbmEst.existenceProbs;
                estimate.cardinality = size(mbmEst.states, 2);
            end
        end
        
        function obj = prune(obj)
            % PRUNE 剪枝操作
            
            % 泊松分量剪枝已在更新步骤中完成
            % MBM分量剪枝已在更新步骤中完成
        end
    end
    
    methods (Access = protected)
        function [zGated, measIdx, trackIdx] = gating(obj, measurement)
            % GATING 门控

            zGated = measurement;
            measIdx = 1:size(measurement, 2);
            trackIdx = [];

            if isempty(obj.MBMComponent.Tracks)
                return;
            end

            % 对每个航迹进行门控
            threshold = obj.Config.gatingThreshold;
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;

            inGate = false(1, size(measurement, 2));

            for i = 1:length(obj.MBMComponent.Tracks)
                track = obj.MBMComponent.Tracks{i};

                % 预测量测
                z_pred = H * track.state;

                % 新息协方差
                S = H * track.covariance * H' + R;

                % 计算马氏距离
                for j = 1:size(measurement, 2)
                    innov = measurement(:, j) - z_pred;
                    d2 = innov' * (S \ innov);

                    if d2 < threshold
                        inGate(j) = true;
                    end
                end
            end

            zGated = measurement(:, inGate);
            measIdx = find(inGate);
        end
        
        function obj = createTracksFromPoisson(obj, measurement, H, R, pD, lambdaC)
            % CREATETRACKSFROMPOISSON 从泊松分量创建新航迹

            if isempty(measurement)
                return;
            end

            for j = 1:size(measurement, 2)
                obj.MBMComponent = obj.MBMComponent.createTrackFromMeasurement(...
                    measurement(:, j), H, R, pD, lambdaC);
            end
        end
    end
end
