classdef ErrorCode
    % ERRORCODE 统一错误码定义
    %
    % 定义多目标跟踪工具箱中使用的所有错误码
    %

    properties (Constant)
        % 成功
        SUCCESS                 = 0
        
        % 输入错误 (1000-1999)
        INVALID_INPUT           = 1000
        MISSING_PARAMETER       = 1001
        INVALID_PARAMETER_TYPE  = 1002
        INVALID_PARAMETER_VALUE = 1003
        DIMENSION_MISMATCH      = 1004
        EMPTY_INPUT             = 1005
        
        % 配置错误 (2000-2999)
        INVALID_CONFIG          = 2000
        MISSING_CONFIG          = 2001
        INCOMPATIBLE_CONFIG     = 2002
        CONFIG_VALIDATION_FAILED = 2003
        
        % 算法错误 (3000-3999)
        NUMERICAL_INSTABILITY   = 3000
        SINGULAR_MATRIX         = 3001
        CONVERGENCE_FAILURE     = 3002
        MEMORY_OVERFLOW         = 3003
        ALGORITHM_FAILURE       = 3004
        INVALID_STATE           = 3005
        
        % 数据关联错误 (4000-4999)
        ASSIGNMENT_FAILURE      = 4000
        NO_VALID_ASSIGNMENT     = 4001
        GATING_FAILURE          = 4002
        
        % 度量计算错误 (5000-5999)
        METRIC_COMPUTATION_ERROR = 5000
        INVALID_REFERENCE       = 5001
        TRAJECTORY_COMPARISON_ERROR = 5002
        
        % 系统错误 (9000-9999)
        FILE_NOT_FOUND          = 9000
        PERMISSION_DENIED       = 9001
        SYSTEM_ERROR            = 9002
        UNSUPPORTED_OPERATION   = 9003
        GENERAL_ERROR           = 9004
    end
    
    methods (Static)
        function msg = getMessage(errorCode)
            % GETMESSAGE 获取错误码对应的错误消息
            %
            % 输入:
            %   errorCode - 错误码
            %
            % 输出:
            %   msg - 错误消息
            
            switch errorCode
                case tracking.core.ErrorCode.SUCCESS
                    msg = '操作成功';
                    
                % 输入错误
                case tracking.core.ErrorCode.INVALID_INPUT
                    msg = '输入参数无效';
                case tracking.core.ErrorCode.MISSING_PARAMETER
                    msg = '缺少必要参数';
                case tracking.core.ErrorCode.INVALID_PARAMETER_TYPE
                    msg = '参数类型错误';
                case tracking.core.ErrorCode.INVALID_PARAMETER_VALUE
                    msg = '参数值无效';
                case tracking.core.ErrorCode.DIMENSION_MISMATCH
                    msg = '维度不匹配';
                case tracking.core.ErrorCode.EMPTY_INPUT
                    msg = '输入为空';
                    
                % 配置错误
                case tracking.core.ErrorCode.INVALID_CONFIG
                    msg = '配置无效';
                case tracking.core.ErrorCode.MISSING_CONFIG
                    msg = '缺少配置';
                case tracking.core.ErrorCode.INCOMPATIBLE_CONFIG
                    msg = '配置不兼容';
                case tracking.core.ErrorCode.CONFIG_VALIDATION_FAILED
                    msg = '配置验证失败';
                    
                % 算法错误
                case tracking.core.ErrorCode.NUMERICAL_INSTABILITY
                    msg = '数值不稳定';
                case tracking.core.ErrorCode.SINGULAR_MATRIX
                    msg = '奇异矩阵';
                case tracking.core.ErrorCode.CONVERGENCE_FAILURE
                    msg = '收敛失败';
                case tracking.core.ErrorCode.MEMORY_OVERFLOW
                    msg = '内存溢出';
                case tracking.core.ErrorCode.ALGORITHM_FAILURE
                    msg = '算法执行失败';
                case tracking.core.ErrorCode.INVALID_STATE
                    msg = '无效状态';
                    
                % 数据关联错误
                case tracking.core.ErrorCode.ASSIGNMENT_FAILURE
                    msg = '数据关联失败';
                case tracking.core.ErrorCode.NO_VALID_ASSIGNMENT
                    msg = '无有效分配';
                case tracking.core.ErrorCode.GATING_FAILURE
                    msg = '门控失败';
                    
                % 度量计算错误
                case tracking.core.ErrorCode.METRIC_COMPUTATION_ERROR
                    msg = '度量计算错误';
                case tracking.core.ErrorCode.INVALID_REFERENCE
                    msg = '无效参考数据';
                case tracking.core.ErrorCode.TRAJECTORY_COMPARISON_ERROR
                    msg = '轨迹比较错误';
                    
                % 系统错误
                case tracking.core.ErrorCode.FILE_NOT_FOUND
                    msg = '文件未找到';
                case tracking.core.ErrorCode.PERMISSION_DENIED
                    msg = '权限被拒绝';
                case tracking.core.ErrorCode.SYSTEM_ERROR
                    msg = '系统错误';
                case tracking.core.ErrorCode.UNSUPPORTED_OPERATION
                    msg = '不支持的操作';
                case tracking.core.ErrorCode.GENERAL_ERROR
                    msg = '通用错误';

                    
                otherwise
                    msg = sprintf('未知错误码: %d', errorCode);
            end
        end
    end
end
