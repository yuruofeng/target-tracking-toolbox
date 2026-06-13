classdef TrackingVisualizer < handle
    % TRACKINGVISUALIZER 跟踪性能可视化类
    %
    % 用于可视化多目标跟踪性能指标
    %
    % 使用示例:
    %   visualizer = viz.TrackingVisualizer();
    %   visualizer.plotMetricTrend(metrics, timeSteps);
    %   visualizer.plotMetricComparison(metrics);
    %   visualizer.plotTrackingResults(estimates, groundTruth);
    %

    properties
        FigureHandle = []  % 图形句柄
        LineStyles = {'-', '--', ':', '-.'}  % 线条样式
        Colors = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.9290 0.6940 0.1250], [0.4940 0.1840 0.5560]}  % 颜色
    end
    
    methods
        function obj = TrackingVisualizer()
            % TRACKINGVISUALIZER 构造函数
        end
        
        function plotMetricTrend(obj, metrics, timeSteps)
            % PLOTMETRICTREND 绘制跟踪指标随时间变化的趋势图
            %
            % 输入:
            %   metrics - 性能指标数据，结构为: metrics{filterIndex}.{metricName}(timeSteps)
            %   timeSteps - 时间步长
            %
            
            % 创建图形
            obj.FigureHandle = figure('Name', '跟踪指标随时间变化趋势', 'Position', [100, 100, 1200, 800]);
            
            % 确定子图布局
            numMetrics = 4;  % GOSPA, 定位, 漏检, 虚警
            [rows, cols] = deal(2, 2);
            
            % 指标名称
            metricNames = {'GOSPA距离', '定位误差', '漏检误差', '虚警误差'};
            metricFields = {'distance', 'localisation', 'missed', 'false'};
            
            % 绘制每个指标的趋势
            for i = 1:numMetrics
                subplot(rows, cols, i);
                hold on;
                
                % 为每个滤波器绘制曲线
                for j = 1:length(metrics)
                    if isfield(metrics{j}, metricFields{i})
                        plot(timeSteps, metrics{j}.(metricFields{i}), ...
                            'LineStyle', obj.LineStyles{mod(j-1, length(obj.LineStyles))+1}, ...
                            'Color', obj.Colors{mod(j-1, length(obj.Colors))+1}, ...
                            'LineWidth', 2, ...
                            'DisplayName', metrics{j}.filterName);
                    end
                end
                
                title(metricNames{i});
                xlabel('时间步');
                ylabel('误差值');
                grid on;
                legend('Location', 'Best');
            end
            
            % 调整布局
            % tightlayout; % 移除对tightlayout的调用，以兼容不同MATLAB版本
        end
        
        function plotMetricComparison(obj, metrics)
            % PLOTMETRICCOMPARISON 绘制各指标数值的柱状对比图
            %
            % 输入:
            %   metrics - 性能指标数据，结构为: metrics{filterIndex}.{metricName}(timeSteps)
            %
            
            % 创建图形
            obj.FigureHandle = figure('Name', '各指标数值对比', 'Position', [100, 100, 1200, 800]);
            
            % 确定子图布局
            numMetrics = 4;  % GOSPA, 定位, 漏检, 虚警
            [rows, cols] = deal(2, 2);
            
            % 指标名称
            metricNames = {'GOSPA距离', '定位误差', '漏检误差', '虚警误差'};
            metricFields = {'distance', 'localisation', 'missed', 'false'};
            
            % 绘制每个指标的对比图
            for i = 1:numMetrics
                subplot(rows, cols, i);
                
                % 准备数据
                filterNames = {};
                metricValues = [];
                
                for j = 1:length(metrics)
                    if isfield(metrics{j}, metricFields{i})
                        filterNames{end+1} = metrics{j}.filterName;
                        % 计算平均值
                        metricValues(end+1) = mean(metrics{j}.(metricFields{i}));
                    end
                end
                
                % 绘制柱状图
                bar(metricValues);
                set(gca, 'XTickLabel', filterNames, 'XTick', 1:length(filterNames));
                xtickangle(45);
                
                title(metricNames{i});
                ylabel('平均误差值');
                grid on;
            end
            
            % 调整布局
            % tightlayout; % 移除对tightlayout的调用，以兼容不同MATLAB版本
        end
        
        function plotTrackingResults(obj, estimates, groundTruth, timeStep)
            % PLOTTRACKINGRESULTS 绘制跟踪结果与真实值的可视化对比
            %
            % 输入:
            %   estimates - 估计结果，结构为: estimates{filterIndex}.states(timeStep)
            %   groundTruth - 真实值，结构为: groundTruth.states(timeStep)
            %   timeStep - 要绘制的时间步
            %
            
            % 创建图形
            obj.FigureHandle = figure('Name', ['跟踪结果与真实值对比 (时间步: ', num2str(timeStep), ')'], 'Position', [100, 100, 1200, 800]);
            
            % 绘制真实值
            if isfield(groundTruth, 'states') && ~isempty(groundTruth.states{timeStep})
                trueStates = groundTruth.states{timeStep};
                plot(trueStates(1, :), trueStates(2, :), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '真实值');
            end
            
            hold on;
            
            % 绘制每个滤波器的估计结果
            for i = 1:length(estimates)
                if isfield(estimates{i}, 'states') && ~isempty(estimates{i}.states{timeStep})
                    estStates = estimates{i}.states{timeStep};
                    plot(estStates(1, :), estStates(2, :), ...
                        'Marker', 'o', ...
                        'MarkerSize', 8, ...
                        'MarkerFaceColor', obj.Colors{mod(i-1, length(obj.Colors))+1}, ...
                        'Color', obj.Colors{mod(i-1, length(obj.Colors))+1}, ...
                        'DisplayName', [estimates{i}.filterName, ' 估计']);
                end
            end
            
            title(['跟踪结果与真实值对比 (时间步: ', num2str(timeStep), ')']);
            xlabel('X 坐标');
            ylabel('Y 坐标');
            legend('Location', 'Best');
            grid on;
            axis equal;
        end
        
        function exportFigures(obj, directory)
            % EXPORTFIGURES 导出图形到指定目录
            %
            % 输入:
            %   directory - 导出目录
            %
            
            if ~exist(directory, 'dir')
                mkdir(directory);
            end
            
            if ~isempty(obj.FigureHandle)
                % 保存当前图形
                filename = fullfile(directory, ['figure_', datestr(now, 'yyyyMMdd_HHmmss'), '.png']);
                print(obj.FigureHandle, filename, '-dpng', '-r300');
                fprintf('图形已导出到: %s\n', filename);
            end
        end
    end
    
    methods (Static)
        function visualizePerformance(metrics, groundTruth, estimates, timeSteps)
            % VISUALIZEPERFORMANCE 静态方法 - 可视化性能指标
            %
            % 输入:
            %   metrics - 性能指标数据
            %   groundTruth - 真实值
            %   estimates - 估计结果
            %   timeSteps - 时间步长
            %
            
            visualizer = viz.TrackingVisualizer();
            
            % 绘制指标趋势图
            visualizer.plotMetricTrend(metrics, timeSteps);
            
            % 绘制指标对比图
            visualizer.plotMetricComparison(metrics);
            
            % 绘制跟踪结果对比 (选择中间时间步)
            if ~isempty(timeSteps)
                middleTimeStep = floor((timeSteps(1) + timeSteps(end)) / 2);
                visualizer.plotTrackingResults(estimates, groundTruth, middleTimeStep);
            end
        end
    end
end