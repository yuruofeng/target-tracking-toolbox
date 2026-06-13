function [mp,Fm] = LinearVelocityConstantTurnrate(m,Ts)

% Motion model from eq (62) in
% Survey of Maneuvering Target Tracking, Part I: Dynamic Models
% Li, Jilkov

% Position and velocity states
xk = m(1:4);
% Turn rate
wk = m(5);

% Pre-compute some quantities
wT = wk*Ts;
sinwT = sin(wT);
coswT = cos(wT);

if abs(wk) > (1e-9)*pi/180
    % If the turn rate is large enough
    Fct = [
        1  0  sinwT/wk      -(1-coswT)/wk;
        0  1  (1-coswT)/wk  sinwT/wk;
        0  0  coswT         -sinwT;
        0  0  sinwT         coswT
        ];
    % Analytic derivatives
    dsinc_dw = (coswT*wT-sinwT)/(wk^2);
    dcosc_dw = (sinwT*wT-1+coswT)/(wk^2);
else
    % Turn rate too small for numerics
    Fct = [
        1 0 Ts 0;
        0 1 0  Ts;
        0 0 1  0;
        0 0 0  1
        ];
    % Derivatives approximated by Taylor expansion
    dsinc_dw = 0;
    dcosc_dw = 0.5*(Ts^2);
end
% Motion model
mp = [Fct*xk; wk];
% Derivatives
dFct_dw = [
    dsinc_dw  -dcosc_dw;
    dcosc_dw  dsinc_dw;
    -sinwT    -coswT;
    coswT     -sinwT
    ]*xk([3 4]);
% Jacobian
Fm = [
    Fct        dFct_dw;
    zeros(1,4) 1
    ];    