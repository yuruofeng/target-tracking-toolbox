classdef MTTException < MException
    % MTTEXCEPTION 多目标跟踪工具箱自定义异常类
    %
    % 继承自MException，添加错误码和上下文信息
    %
    % 使用示例:
    %   ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_INPUT, ...
    %       '参数 %s 的值无效', paramName);
    %   throw(ME);
    %

    properties
        ErrorCode    double      % 错误码
        ErrorLevel   char = 'error'  % 错误级别: 'error', 'warning', 'info'
        Context      struct = struct()  % 错误上下文信息
    end
    
    methods
        function obj = MTTException(errorCode, message, varargin)
            % MTTEXCEPTION 构造函数
            %
            % 输入:
            %   errorCode - 错误码 (使用tracking.core.ErrorCode常量)
            %   message   - 错误消息 (支持格式化字符串)
            %   varargin  - 格式化参数
            
            % 获取错误消息
            if ~isempty(varargin)
                formattedMsg = sprintf(message, varargin{:});
            else
                formattedMsg = message;
            end
            
            % 添加错误码信息
            errorMsg = tracking.core.ErrorCode.getMessage(errorCode);
            fullMsg = sprintf('[%d] %s: %s', errorCode, errorMsg, formattedMsg);
            
            % 创建标识符 (使用有效的 MATLAB 消息标识符格式)
            identifier = sprintf('MTT:Error%d', errorCode);
            
            % 调用父类构造函数
            obj = obj@MException(identifier, fullMsg);
            
            % 设置错误码
            obj.ErrorCode = errorCode;
        end
        
        function obj = setContext(obj, key, value)
            % SETCONTEXT 设置上下文信息
            %
            % 输入:
            %   key   - 上下文键名
            %   value - 上下文值
            %
            % 输出:
            %   obj - 修改后的异常对象
            %
            % 示例:
            %   ME = ME.setContext('function', 'GMPHD_filter');
            %   ME = ME.setContext('timestamp', k);
            
            obj.Context.(key) = value;
        end
        
        function obj = addContext(obj, contextStruct)
            % ADDCONTEXT 添加多个上下文信息
            %
            % 输入:
            %   contextStruct - 包含多个上下文字段的结构体
            %
            % 输出:
            %   obj - 修改后的异常对象
            
            fields = fieldnames(contextStruct);
            for i = 1:length(fields)
                obj.Context.(fields{i}) = contextStruct.(fields{i});
            end
        end
        
        function display(obj)
            % DISPLAY 显示异常详细信息
            
            fprintf('\n==== MTT异常信息 ====\n');
            fprintf('错误码: %d\n', obj.ErrorCode);
            fprintf('错误级别: %s\n', obj.ErrorLevel);
            fprintf('标识符: %s\n', obj.identifier);
            fprintf('消息: %s\n', obj.message);
            
            % 显示上下文信息
            if ~isempty(fieldnames(obj.Context))
                fprintf('\n上下文信息:\n');
                fields = fieldnames(obj.Context);
                for i = 1:length(fields)
                    value = obj.Context.(fields{i});
                    if isnumeric(value) && isscalar(value)
                        fprintf('  %s: %g\n', fields{i}, value);
                    else
                        fprintf('  %s: %s\n', fields{i}, evalc('disp(value)'));
                    end
                end
            end
            
            % 显示堆栈信息
            if ~isempty(obj.stack)
                fprintf('\n堆栈跟踪:\n');
                for i = 1:length(obj.stack)
                    fprintf('  [%d] %s (line %d)\n', i, ...
                        obj.stack(i).name, obj.stack(i).line);
                end
            end
            
            fprintf('====================\n\n');
        end
    end
    
    methods (Static)
        function throwInvalidInput(paramName, expectedType, actualType)
            % THROWINVALIDINPUT 抛出输入无效异常
            %
            % 输入:
            %   paramName    - 参数名
            %   expectedType - 期望类型
            %   actualType   - 实际类型
            
            ME = tracking.core.MTTException(tracking.core.ErrorCode.INVALID_INPUT, ...
                '参数 "%s" 类型错误，期望 %s，实际为 %s', ...
                paramName, expectedType, actualType);
            throw(ME);
        end
        
        function throwMissingParameter(paramName)
            % THROWMISSINGPARAMETER 抛出缺少参数异常
            %
            % 输入:
            %   paramName - 参数名
            
            ME = tracking.core.MTTException(tracking.core.ErrorCode.MISSING_PARAMETER, ...
                '缺少必要参数: %s', paramName);
            throw(ME);
        end
        
        function throwDimensionMismatch(expectedDim, actualDim, context)
            % THROWDIMENSIONMISMATCH 抛出维度不匹配异常
            %
            % 输入:
            %   expectedDim - 期望维度
            %   actualDim   - 实际维度
            %   context     - (可选) 上下文信息
            
            if nargin < 3
                context = '';
            end
            
            ME = tracking.core.MTTException(tracking.core.ErrorCode.DIMENSION_MISMATCH, ...
                '维度不匹配: 期望 %s, 实际 %s. %s', ...
                mat2str(expectedDim), mat2str(actualDim), context);
            
            if nargin >= 3
                ME = ME.setContext('expected', expectedDim);
                ME = ME.setContext('actual', actualDim);
            end
            
            throw(ME);
        end
        
        function throwNumericalError(operation, details)
            % THROWNUMERICALERROR 抛出数值计算错误
            %
            % 输入:
            %   operation - 操作名称
            %   details   - 错误详情
            
            ME = tracking.core.MTTException(tracking.core.ErrorCode.NUMERICAL_INSTABILITY, ...
                '数值计算错误在操作 "%s": %s', operation, details);
            ME = ME.setContext('operation', operation);
            throw(ME);
        end
    end
end
