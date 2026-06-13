classdef GMTPHD < tracking.multi.rfs.core.BaseFilter
    % GMTPHD 高斯混合轨迹PHD滤波器
    %
    % 实现基于轨迹的PHD滤波器
    %
    % 参考文献:
    %   A. F. García-Fernández and L. Svensson, "Trajectory PHD and CPHD filters",
    %   IEEE Transactions on Signal Processing, vol. 67, no. 22, pp. 5702-5714,
    %   Nov. 2019.
    %
    % 使用示例:
    %   config = tracking.multi.rfs.core.FilterConfig('motionModel', struct('type', 'CV', 'F', F, 'Q', Q), ...
    %                               'measurementModel', struct('type', 'Linear', 'H', H, 'R', R), ...
    %                               'pruningThreshold', 1e-4, ...
    %                               'absorptionThreshold', 4, ...
    %                               'maxComponents', 30, ...
    %                               'Lscan', 5);
    %   filter = tracking.multi.rfs.trajectory.phd.GMTPHD(config);
    %   result = filter.run(measurements, groundTruth);
    %

    properties (Access = private)
        % 滤波器状态
        Weights       % PHD权重
        Means         % PHD均值 (L-scan窗口内)
        Covariances   % PHD协方差 (L-scan窗口内)
        LogicalActives % 活跃组件标志
        InitialTimes  % 轨迹初始时间
        Lengths       % 轨迹长度
        OldMeans      % L-scan窗口外的均值
        Lscan         % L-scan窗口长度
        MaxComponents % 最大组件数
        AbsorptionThreshold % 吸收阈值
    end
    
    methods
        function obj = GMTPHD(config)
            % GMTPHD 构造函数
            %
            % 输入:
            %   config - FilterConfig对象
            
            % 调用基类构造函数
            obj = obj@tracking.multi.rfs.core.BaseFilter(config);
            
            % 初始化TPHD特定参数
            if isfield(config.extraParams, 'Lscan')
                obj.Lscan = config.extraParams.Lscan;
            else
                obj.Lscan = 5;
            end

            if isfield(config.extraParams, 'maxComponents')
                obj.MaxComponents = config.extraParams.maxComponents;
            else
                obj.MaxComponents = config.maxComponents;
            end

            if isfield(config.extraParams, 'absorptionThreshold')
                obj.AbsorptionThreshold = config.extraParams.absorptionThreshold;
            else
                obj.AbsorptionThreshold = 4;
            end
            
            % 初始化状态
            obj.Weights = zeros(1, obj.MaxComponents);
            obj.Means = zeros(obj.Lscan * size(config.motionModel.F, 1), obj.MaxComponents);
            obj.Covariances = zeros(obj.Lscan * size(config.motionModel.F, 1), ...
                obj.Lscan * size(config.motionModel.F, 1), obj.MaxComponents);
            obj.LogicalActives = false(1, obj.MaxComponents);
            obj.InitialTimes = zeros(1, obj.MaxComponents);
            obj.Lengths = zeros(1, obj.MaxComponents);
            obj.OldMeans = [];
        end
        
        function obj = initialize(obj)
            % INITIALIZE 初始化滤波器
            
            % 重置状态
            obj.Weights = zeros(1, obj.MaxComponents);
            obj.Means = zeros(obj.Lscan * size(obj.Config.motionModel.F, 1), obj.MaxComponents);
            obj.Covariances = zeros(obj.Lscan * size(obj.Config.motionModel.F, 1), ...
                obj.Lscan * size(obj.Config.motionModel.F, 1), obj.MaxComponents);
            obj.LogicalActives = false(1, obj.MaxComponents);
            obj.InitialTimes = zeros(1, obj.MaxComponents);
            obj.Lengths = zeros(1, obj.MaxComponents);
            obj.OldMeans = [];
            obj.CurrentTime = 0;
        end
        
        function obj = predict(obj)
            % PREDICT 预测步骤
            
            if obj.CurrentTime == 0
                return;
            end
            
            % 获取活跃组件
            activeIndices = find(obj.LogicalActives);
            Ncom = length(activeIndices);
            
            if Ncom == 0
                return;
            end
            
            % 提取参数
            F = obj.Config.motionModel.F;
            Q = obj.Config.motionModel.Q;
            p_s = obj.Config.survivalProb;
            Nx = size(F, 1);
            
            % 对每个组件进行预测
            for i = 1:Ncom
                idx = activeIndices(i);
                mean_i = obj.Means(:, idx);
                cov_i = obj.Covariances(:, :, idx);
                length_i = obj.Lengths(idx);
                
                if length_i < obj.Lscan + 1
                    % 轨迹长度小于L-scan窗口
                    indexActual = Nx * length_i - (Nx - 1) : Nx * length_i;
                    indexPrev = indexActual - Nx;
                    
                    % 预测均值
                    meanPred = mean_i;
                    meanPred(indexActual) = F * meanPred(indexPrev);
                    
                    % 预测协方差
                    covAnt = cov_i(1:indexActual(1)-1, 1:indexActual(1)-1);
                    Fmode = zeros(Nx, size(covAnt, 1));
                    Fmode(end-(Nx-1):end, end-(Nx-1):end) = F;
                    cross = Fmode * covAnt;
                    
                    covPred = cov_i;
                    covPred(indexActual, indexActual) = F * covPred(indexPrev, indexPrev) * F' + Q;
                    covPred(1:indexActual(1)-1, indexActual) = cross';
                    covPred(indexActual, 1:indexActual(1)-1) = cross;
                else
                    % 调整均值和协方差到L-scan窗口
                    meanPred = zeros(Nx * obj.Lscan, 1);
                    meanPred(end-(Nx-1):end) = F * mean_i(end-(Nx-1):end);
                    meanPred(1:end-Nx) = mean_i(Nx+1:end);
                    
                    % 保存旧均值
                    if size(obj.OldMeans, 1) < Nx * obj.CurrentTime
                        obj.OldMeans = [obj.OldMeans; zeros(Nx, size(obj.OldMeans, 2))];
                    end
                    obj.OldMeans(Nx*obj.CurrentTime - Nx*obj.Lscan + 1 : Nx*obj.CurrentTime - Nx*obj.Lscan + Nx, idx) = mean_i(1:Nx);
                    
                    % 预测协方差
                    covAnt = cov_i(Nx+1:end, Nx+1:end);
                    covPred = zeros(Nx * obj.Lscan, Nx * obj.Lscan);
                    covPred(end-(Nx-1):end, end-(Nx-1):end) = F * cov_i(end-(Nx-1):end, end-(Nx-1):end) * F' + Q;
                    
                    if obj.Lscan > 1
                        Fmode = zeros(Nx, Nx*(obj.Lscan-1));
                        Fmode(:, end-(Nx-1):end) = F;
                        cross = Fmode * covAnt;
                        covPred(end-(Nx-1):end, 1:end-Nx) = cross;
                        covPred(1:end-Nx, end-(Nx-1):end) = cross';
                        covPred(1:end-Nx, 1:end-Nx) = cov_i(Nx+1:end, Nx+1:end);
                    end
                end
                
                % 更新权重（考虑生存概率）
                obj.Weights(idx) = obj.Weights(idx) * p_s;
                obj.Means(:, idx) = meanPred;
                obj.Covariances(:, :, idx) = covPred;
                obj.Lengths(idx) = obj.Lengths(idx) + 1;
            end
        end
        
        function obj = update(obj, measurement)
            % UPDATE 更新步骤
            %
            % 输入:
            %   measurement - 当前时刻的测量
            
            % 提取参数
            H = obj.Config.measurementModel.H;
            R = obj.Config.measurementModel.R;
            p_d = obj.Config.detectionProb;
            lambda_c = obj.Config.clutterRate / prod(obj.Config.surveillanceArea);
            area = obj.Config.surveillanceArea;
            Nx = size(obj.Config.motionModel.F, 1);
            Nz = size(H, 1);
            
            % 计算当前活跃组件数
            activeIndices = find(obj.LogicalActives);
            Ncom = length(activeIndices);
            
            if Ncom == 0
                return;
            end
            
            % 预计算卡尔曼滤波器参数
            zPredSeries = zeros(Nz, 1, Ncom);
            SPredSeries = zeros(Nz, Nz, Ncom);
            KPredSeries = zeros(Nx * obj.Lscan, Nz, Ncom);
            PUSeries = zeros(Nx * obj.Lscan, Nx * obj.Lscan, Ncom);
            
            for i = 1:Ncom
                idx = activeIndices(i);
                length_i = obj.Lengths(idx);
                
                % 确定索引
                if length_i < obj.Lscan
                    indexActual = Nx * length_i - (Nx - 1) : Nx * length_i;
                    indexTrajectory = 1 : Nx * length_i;
                else
                    indexActual = Nx * obj.Lscan - (Nx - 1) : Nx * obj.Lscan;
                    indexTrajectory = 1 : Nx * obj.Lscan;
                end
                
                % 预测测量
                zPred = H * obj.Means(indexActual, idx);
                
                % 构造轨迹测量矩阵
                minLength = min([obj.Lscan, length_i]);
                HTrajectory = [zeros(Nz, Nx*(minLength-1)), H];
                
                % 预测测量协方差
                cov_i = obj.Covariances(indexTrajectory, indexTrajectory, idx);
                SPred = HTrajectory * cov_i * HTrajectory' + R;
                SPred = (SPred + SPred') / 2;
                
                % 计算卡尔曼增益
                KPred = cov_i * HTrajectory' / SPred;
                PU = (eye(length(indexTrajectory)) - KPred * HTrajectory) * cov_i;
                PU = (PU + PU') / 2;
                
                % 存储结果
                zPredSeries(:, 1, i) = zPred;
                SPredSeries(:, :, i) = SPred;
                KPredSeries(indexTrajectory, :, i) = KPred;
                PUSeries(indexTrajectory, indexTrajectory, i) = PU;
            end
            
            % 计算更新后的组件数
            Nmeasurements = size(measurement, 2);
            NcomU = Nmeasurements * Ncom + Ncom;
            
            % 初始化更新后的参数
            weightsU = zeros(1, obj.MaxComponents);
            meansU = zeros(size(obj.Means));
            covsU = zeros(size(obj.Covariances));
            tIniU = zeros(1, obj.MaxComponents);
            lengthU = zeros(1, obj.MaxComponents);
            logicalActivesU = false(1, obj.MaxComponents);
            logicalActivesU(1:NcomU) = true;
            oldMeansU = obj.OldMeans;
            
            % 更新未检测到的组件
            indexU = 1;
            for i = 1:Ncom
                idx = activeIndices(i);
                weight = (1 - p_d) * obj.Weights(idx);
                mean = obj.Means(:, idx);
                cov = obj.Covariances(:, :, idx);
                tIni = obj.InitialTimes(idx);
                length_i = obj.Lengths(idx);
                
                weightsU(indexU) = weight;
                meansU(:, indexU) = mean;
                covsU(:, :, indexU) = cov;
                tIniU(indexU) = tIni;
                lengthU(indexU) = length_i;
                
                if size(oldMeansU, 2) >= indexU
                    oldMeansU(:, indexU) = obj.OldMeans(:, idx);
                end
                
                indexU = indexU + 1;
            end
            
            % 更新检测到的组件
            for p = 1:Nmeasurements
                sumWeights = 0;
                for i = 1:Ncom
                    idx = activeIndices(i);
                    length_i = obj.Lengths(idx);
                    
                    % 确定索引
                    if length_i < obj.Lscan
                        indexTrajectory = 1 : Nx * length_i;
                    else
                        indexTrajectory = 1 : Nx * obj.Lscan;
                    end
                    
                    tIni = obj.InitialTimes(idx);
                    
                    % 计算权重
                    gaussianEv = obj.evaluateGaussian(measurement(:, p), zPredSeries(:, 1, i), SPredSeries(:, :, i));
                    weight_ip = p_d * obj.Weights(idx) * gaussianEv;
                    
                    % 更新均值和协方差
                    mean_ip = obj.Means(indexTrajectory, idx) + KPredSeries(indexTrajectory, :, i) * (measurement(:, p) - zPredSeries(:, 1, i));
                    cov_ip = PUSeries(indexTrajectory, indexTrajectory, i);
                    
                    % 存储结果
                    weightsU(indexU) = weight_ip;
                    meansU(indexTrajectory, indexU) = mean_ip;
                    covsU(:, :, indexU) = cov_ip;
                    tIniU(indexU) = tIni;
                    lengthU(indexU) = length_i;
                    
                    if size(oldMeansU, 2) >= indexU
                        oldMeansU(:, indexU) = obj.OldMeans(:, idx);
                    end
                    
                    indexU = indexU + 1;
                    sumWeights = sumWeights + weight_ip;
                end
                
                % 归一化权重
                intensityClutter = lambda_c / (area(1) * area(2));
                weightsU(indexU - Ncom : indexU - 1) = weightsU(indexU - Ncom : indexU - 1) / (intensityClutter + sumWeights);
            end
            
            % 更新状态
            obj.Weights = weightsU;
            obj.Means = meansU;
            obj.Covariances = covsU;
            obj.InitialTimes = tIniU;
            obj.Lengths = lengthU;
            obj.LogicalActives = logicalActivesU;
            obj.OldMeans = oldMeansU;
        end
        
        function estimate = estimate(obj)
            % ESTIMATE 估计步骤
            %
            % 输出:
            %   estimate - 估计结果
            
            % 提取活跃组件
            activeIndices = find(obj.LogicalActives);
            Ncom = length(activeIndices);
            
            if Ncom == 0
                estimate = struct('states', [], 'weights', [], 'cardinality', 0);
                return;
            end
            
            % 简单的估计实现
            % 实际应用中应该使用更复杂的估计方法
            weights = obj.Weights(activeIndices);
            means = obj.Means(:, activeIndices);
            Nx = size(obj.Config.motionModel.F, 1);
            
            % 提取每个组件的最新状态
            states = zeros(Nx, Ncom);
            for i = 1:Ncom
                length_i = obj.Lengths(activeIndices(i));
                if length_i <= obj.Lscan
                    index = Nx * length_i - (Nx - 1) : Nx * length_i;
                else
                    index = Nx * obj.Lscan - (Nx - 1) : Nx * obj.Lscan;
                end
                states(:, i) = means(index, i);
            end
            
            % 计算 cardinality 估计
            cardinality = sum(weights);
            
            estimate = struct('states', states, 'weights', weights, 'cardinality', cardinality);
        end
        
        function obj = prune(obj)
            % PRUNE 剪枝和吸收步骤
            
            % 提取活跃组件
            activeIndices = find(obj.LogicalActives);
            Ncom = length(activeIndices);
            
            if Ncom == 0
                return;
            end
            
            % 剪枝：移除权重低于阈值的组件
            weights = obj.Weights(activeIndices);
            keepIndices = weights > obj.Config.pruningThreshold;
            activeIndices = activeIndices(keepIndices);
            Ncom = length(activeIndices);
            
            if Ncom == 0
                obj.LogicalActives = false(1, obj.MaxComponents);
                return;
            end
            
            % 吸收：合并相似的组件
            % 这里实现一个简单的吸收逻辑
            means = obj.Means(:, activeIndices);
            covs = obj.Covariances(:, :, activeIndices);
            weights = obj.Weights(activeIndices);
            tInis = obj.InitialTimes(activeIndices);
            lengths = obj.Lengths(activeIndices);
            oldMeans = obj.OldMeans(:, activeIndices);
            
            % 按权重排序
            [sortedWeights, sortedIndices] = sort(weights, 'descend');
            sortedMeans = means(:, sortedIndices);
            sortedCovs = covs(:, :, sortedIndices);
            sortedTInis = tInis(sortedIndices);
            sortedLengths = lengths(sortedIndices);
            sortedOldMeans = oldMeans(:, sortedIndices);
            
            % 执行吸收
            keptIndices = true(1, Ncom);
            for i = 1:Ncom
                if keptIndices(i)
                    for j = i+1:Ncom
                        if keptIndices(j)
                            % 计算组件间的距离
                            length_i = sortedLengths(i);
                            length_j = sortedLengths(j);
                            
                            % 只比较相同长度的轨迹
                            if abs(length_i - length_j) <= 1
                                % 计算均值距离
                                if length_i <= obj.Lscan
                                    index_i = size(obj.Config.motionModel.F, 1) * length_i - (size(obj.Config.motionModel.F, 1) - 1) : size(obj.Config.motionModel.F, 1) * length_i;
                                else
                                    index_i = size(obj.Config.motionModel.F, 1) * obj.Lscan - (size(obj.Config.motionModel.F, 1) - 1) : size(obj.Config.motionModel.F, 1) * obj.Lscan;
                                end
                                
                                if length_j <= obj.Lscan
                                    index_j = size(obj.Config.motionModel.F, 1) * length_j - (size(obj.Config.motionModel.F, 1) - 1) : size(obj.Config.motionModel.F, 1) * length_j;
                                else
                                    index_j = size(obj.Config.motionModel.F, 1) * obj.Lscan - (size(obj.Config.motionModel.F, 1) - 1) : size(obj.Config.motionModel.F, 1) * obj.Lscan;
                                end
                                
                                distance = norm(sortedMeans(index_i, i) - sortedMeans(index_j, j));
                                
                                % 如果距离小于吸收阈值，合并组件
                                if distance < obj.AbsorptionThreshold
                                    sortedWeights(i) = sortedWeights(i) + sortedWeights(j);
                                    keptIndices(j) = false;
                                end
                            end
                        end
                    end
                end
            end
            
            % 保留未被吸收的组件
            keptWeights = sortedWeights(keptIndices);
            keptMeans = sortedMeans(:, keptIndices);
            keptCovs = sortedCovs(:, :, keptIndices);
            keptTInis = sortedTInis(keptIndices);
            keptLengths = sortedLengths(keptIndices);
            keptOldMeans = sortedOldMeans(:, keptIndices);
            
            % 限制最大组件数
            if length(keptWeights) > obj.MaxComponents
                keptWeights = keptWeights(1:obj.MaxComponents);
                keptMeans = keptMeans(:, 1:obj.MaxComponents);
                keptCovs = keptCovs(:, :, 1:obj.MaxComponents);
                keptTInis = keptTInis(1:obj.MaxComponents);
                keptLengths = keptLengths(1:obj.MaxComponents);
                keptOldMeans = keptOldMeans(:, 1:obj.MaxComponents);
            end
            
            % 更新状态
            obj.Weights = zeros(1, obj.MaxComponents);
            obj.Means = zeros(size(obj.Means));
            obj.Covariances = zeros(size(obj.Covariances));
            obj.InitialTimes = zeros(1, obj.MaxComponents);
            obj.Lengths = zeros(1, obj.MaxComponents);
            obj.LogicalActives = false(1, obj.MaxComponents);
            obj.OldMeans = zeros(size(obj.OldMeans, 1), obj.MaxComponents);
            
            Nkept = length(keptWeights);
            if Nkept > 0
                obj.Weights(1:Nkept) = keptWeights;
                obj.Means(:, 1:Nkept) = keptMeans;
                obj.Covariances(:, :, 1:Nkept) = keptCovs;
                obj.InitialTimes(1:Nkept) = keptTInis;
                obj.Lengths(1:Nkept) = keptLengths;
                obj.LogicalActives(1:Nkept) = true;
                obj.OldMeans(:, 1:Nkept) = keptOldMeans;
            end
        end
    end
    
    methods (Access = protected)
        function value = evaluateGaussian(obj, x, mu, sigma)
            % EVALUATEGAUSSIAN 计算高斯概率密度
            %
            % 输入:
            %   x - 输入向量
            %   mu - 均值
            %   sigma - 协方差矩阵
            %
            % 输出:
            %   value - 概率密度值
            
            N = length(x);
            invSigma = inv(sigma);
            detSigma = det(sigma);
            
            if detSigma <= 0
                value = 0;
                return;
            end
            
            diff = x - mu;
            exponent = -0.5 * diff' * invSigma * diff;
            value = (1 / sqrt((2*pi)^N * detSigma)) * exp(exponent);
        end
    end
end
