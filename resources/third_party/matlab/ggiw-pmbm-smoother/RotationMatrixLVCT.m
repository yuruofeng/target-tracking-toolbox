function [M,M1,M2,I_x] = RotationMatrixLVCT(m)

% Assumes five states: 2D position, 2D velocity, and rotation
% Assumes the sample time is 1 second

% States that have any effect on the function M
I_x = 5;

% Rotation matrix
M = [cos(m(5)) -sin(m(5)); sin(m(5)) cos(m(5))];

% First derivative
M1 = [-sin(m(5)) -cos(m(5)); cos(m(5)) -sin(m(5))];

% Second derivative
M2 = [-cos(m(5)) sin(m(5)); -sin(m(5)) -cos(m(5))];