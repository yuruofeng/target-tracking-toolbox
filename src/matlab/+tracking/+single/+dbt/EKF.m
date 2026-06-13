classdef EKF < tracking.single.dbt.KalmanFilterBase
% DBT.EKF  Extended Kalman Filter for Detect-Before-Track.
%   Implements the EKF for tracking targets with CT (Constant Turn-Rate)
%   dynamics and range/azimuth measurements using Jacobian linearization.
%
%   State vector: [x, vx, y, vy, omega]
%       x, y     - Position coordinates [m]
%       vx, vy   - Velocity components [m/s]
%       omega    - Turn rate [rad/s]
%
%   Measurement: [azimuth, range]
%       azimuth  - Bearing angle [rad]
%       range    - Distance to target [m]
%
%   Usage:
%       cfg = tracking.single.dbt.Config();
%       ekf = tracking.single.dbt.EKF(cfg);
%       [state, covar] = ekf.init(x0, P0);
%       for k = 1:numSteps
%           [state, covar] = ekf.predict(state, covar);
%           [state, covar, innov, innovCov] = ekf.update(meas(:,k), state, covar);
%       end
%
%   Methods:
%       init    - Initialize filter state and covariance
%       predict - Time update using CT dynamics
%       update  - Measurement update with Jacobian linearization
%       run     - Execute filter on measurement sequence
%
%   See also: tracking.single.dbt.KalmanFilterBase, tracking.single.dbt.UKF, tracking.single.dbt.CKF, tracking.single.dbt.Config

    methods
        function obj = EKF(cfg)
        % EKF  Create Extended Kalman Filter with configuration.
        %   obj = EKF(cfg) initializes with tracking.single.dbt.Config object.
        %   obj = EKF() uses default configuration.
            obj@tracking.single.dbt.KalmanFilterBase(cfg);
        end
    end

    methods (Access = protected)
        function [statePre, covarPre] = predictStep(obj, stateUpd, covarUpd)
        % PREDICTSTEP  EKF prediction using Jacobian of dynamics.
            F = tracking.models.MeasurementModel.ctDynamicMatrix(obj.config.dt, stateUpd);
            statePre = F * stateUpd;
            covarPre = F * covarUpd * F' + obj.processNoise;
        end
        
        function [stateUpd, covarUpd, innov, innovCov] = updateStep(obj, measZ, statePre, covarPre)
        % UPDATESTEP  EKF update using measurement Jacobian.
            H = tracking.models.MeasurementModel.ctMeasJacobian(statePre);
            measPre = tracking.models.MeasurementModel.ctMeasFunc(statePre);
            
            innov = measZ - measPre;
            innov = obj.wrapAngleInnovation(innov, 1);
            
            innovCov = H * covarPre * H' + obj.measurementNoise;
            K = obj.computeKalmanGain(covarPre, H, innovCov);
            
            stateUpd = statePre + K * innov;
            covarUpd = (eye(obj.stateDim) - K * H) * covarPre;
        end
    end

    methods (Static)
        function name = getFilterType()
            name = 'EKF';
        end
        
        function description = getFilterDescription()
            description = 'Extended Kalman Filter with Jacobian linearization';
        end
    end
end
