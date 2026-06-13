classdef TrajectoryVisualizer < handle
    % TRAJECTORYVISUALIZER 轨迹可视化工具
    %
    % 用于绘制滤波器输出和真值轨迹的可视化工具
    %
    % 使用示例:
    %   viz = viz.TrajectoryVisualizer();
    %   viz.plotTrajectories(X_truth, t_birth, t_death, X_estimate, z, k);
    %
    % 来源: Trajectory errors/DrawTrajectoryFilterEstimates.m

    properties
        FigureHandle            % 图形句柄
        XLimit = [-100 1000]    % X轴范围
        YLimit = [-100 1000]    % Y轴范围
        ShowMeasurements = true % 是否显示测量
        ShowEstimates = true    % 是否显示估计
        ShowGroundTruth = true  % 是否显示真值
        LineWidth = 1.3         % 线宽
        MarkerSize = 8          % 标记大小
    end
    
    properties (Access = private)
        CurrentTime = 0         % 当前时刻
    end
    
    methods
        function obj = TrajectoryVisualizer(varargin)
            % TRAJECTORYVISUALIZER 构造函数
            %
            % 可选参数:
            %   'XLimit' - X轴范围 [min, max]
            %   'YLimit' - Y轴范围 [min, max]
            %   'ShowMeasurements' - 是否显示测量
            %   'ShowEstimates' - 是否显示估计
            %   'ShowGroundTruth' - 是否显示真值
            
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                
                switch lower(key)
                    case 'xlimit'
                        obj.XLimit = value;
                    case 'ylimit'
                        obj.YLimit = value;
                    case 'showmeasurements'
                        obj.ShowMeasurements = value;
                    case 'showestimates'
                        obj.ShowEstimates = value;
                    case 'showgroundtruth'
                        obj.ShowGroundTruth = value;
                end
            end
            
            obj.FigureHandle = figure('Name', 'Trajectory Visualization');
        end
        
        function plotTrajectories(obj, X_truth, t_birth, t_death, X_estimate, z, k)
            % PLOTTRAJECTORIES 绘制轨迹
            %
            % 输入:
            %   X_truth - 真值状态矩阵 (Nx*Ntargets x Nsteps)
            %   t_birth - 出生时刻向量
            %   t_death - 死亡时刻向量
            %   X_estimate - 估计状态 (cell数组)
            %   z - 测量 (2 x Nz)
            %   k - 当前时刻
            
            obj.CurrentTime = k;
            
            clf(obj.FigureHandle);
            axis(obj.FigureHandle, [obj.XLimit(1) obj.XLimit(2), obj.YLimit(1) obj.YLimit(2)]);
            xlabel(obj.FigureHandle, 'x position (m)');
            ylabel(obj.FigureHandle, 'y position (m)');
            grid(obj.FigureHandle, 'on');
            hold(obj.FigureHandle, 'on');
            
            Nx = 4;
            
            if obj.ShowGroundTruth && ~isempty(X_truth)
                numTargets = size(X_truth, 1) / Nx;
                
                for i = 1:numTargets
                    xPlot = X_truth((i-1)*Nx+1, t_birth(i):t_death(i)-1);
                    yPlot = X_truth((i-1)*Nx+3, t_birth(i):t_death(i)-1);
                    
                    plot(obj.FigureHandle, xPlot, yPlot, 'b', 'LineWidth', obj.LineWidth);
                    
                    text(obj.FigureHandle, xPlot(1), yPlot(1), num2str(i), 'color', 'b');
                    
                    if t_birth(i) <= k && k <= t_death(i)-1
                        plot(obj.FigureHandle, X_truth((i-1)*Nx+1, t_birth(i):k), ...
                             X_truth((i-1)*Nx+3, t_birth(i):k), 'b', 'LineWidth', 2);
                    end
                end
            end
            
            if obj.ShowEstimates && ~isempty(X_estimate)
                for i = 1:length(X_estimate)
                    if ~isempty(X_estimate{i})
                        X_est_i = X_estimate{i};
                        xEst = X_est_i(1:4:end);
                        yEst = X_est_i(3:4:end);
                        plot(obj.FigureHandle, xEst, yEst, '-or', 'MarkerSize', obj.MarkerSize);
                    end
                end
            end
            
            if obj.ShowMeasurements && ~isempty(z)
                plot(obj.FigureHandle, z(1,:), z(2,:), 'ok', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
            end
            
            hold(obj.FigureHandle, 'off');
            
            title(obj.FigureHandle, sprintf('Time step: %d', k));
        end
        
        function plotErrorHistory(obj, timeSteps, errors, errorType)
            % PLOTERRORHISTORY 绘制误差历史
            %
            % 输入:
            %   timeSteps - 时间步向量
            %   errors - 误差向量
            %   errorType - 误差类型字符串
            
            figure(obj.FigureHandle);
            plot(timeSteps, errors, 'b', 'LineWidth', obj.LineWidth);
            grid on;
            xlabel('Time step');
            ylabel(errorType);
            title([errorType ' over time']);
        end
        
        function plotGOSPADecomposition(obj, timeSteps, loc, mis, fal)
            % PLOTGOSPADECOMPOSITION 绘制GOSPA分解
            %
            % 输入:
            %   timeSteps - 时间步向量
            %   loc - 定位误差
            %   mis - 漏检误差
            %   fal - 虚警误差
            
            figure(obj.FigureHandle);
            hold on;
            plot(timeSteps, loc, 'r', 'LineWidth', obj.LineWidth, 'DisplayName', 'Localization');
            plot(timeSteps, mis, 'g', 'LineWidth', obj.LineWidth, 'DisplayName', 'Missed');
            plot(timeSteps, fal, 'b', 'LineWidth', obj.LineWidth, 'DisplayName', 'False Alarm');
            hold off;
            grid on;
            xlabel('Time step');
            ylabel('Error');
            title('GOSPA Decomposition');
            legend('Location', 'best');
        end
        
        function fig = getFigure(obj)
            % GETFIGURE 获取图形句柄
            %
            % 输出:
            %   fig - 图形句柄
            
            fig = obj.FigureHandle;
        end
        
        function saveCurrentPlot(obj, filename)
            % SAVECURRENTPLOT 保存当前图形
            %
            % 输入:
            %   filename - 文件名
            
            saveas(obj.FigureHandle, filename);
        end
    end
    
    methods (Static)
        function drawFilterEstimates(X_truth, t_birth, t_death, X_estimate, XLim, YLim, z, k)
            % DRAWFILTERESTIMATES 静态方法：绘制滤波器估计
            %
            % 输入:
            %   X_truth - 真值状态
            %   t_birth - 出生时刻
            %   t_death - 死亡时刻
            %   X_estimate - 估计状态
            %   XLim - X轴范围
            %   YLim - Y轴范围
            %   z - 测量
            %   k - 当前时刻
            
            viz = viz.TrajectoryVisualizer('XLimit', XLim, 'YLimit', YLim);
            viz.plotTrajectories(X_truth, t_birth, t_death, X_estimate, z, k);
        end
    end
end
