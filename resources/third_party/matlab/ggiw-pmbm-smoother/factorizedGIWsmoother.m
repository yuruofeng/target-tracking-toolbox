function [msm,Psm,vsm,Vsm] = factorizedGIWsmoother(mpred,Ppred,vpred,Vpred,mup,Pup,vup,Vup,models)

% Allocate memory
nx = size(mpred,1);
d = size(Vpred,1);
Nt = size(mpred,2);
msm = zeros(nx,Nt);
Psm = zeros(nx,nx,Nt);
vsm = zeros(1,Nt);
Vsm = zeros(d,d,Nt);

% Initialize
msm(:,end) = mup(:,end);
Psm(:,:,end) = Pup(:,:,end);
vsm(end) = vup(end);
Vsm(:,:,end) = Vup(:,:,end);

% Begin from final time step
t = Nt;
% Updated and predicted parameters are equal after the last measurement update 
while vup(t) == vpred(t) && t>1
    % Decrease time
    t = t-1;
    % Smoothed parameters equal to filtered parameters
    msm(:,t) = mup(:,t);
    Psm(:,:,t) = Pup(:,:,t);
    vsm(t) = vup(t);
    Vsm(:,:,t) = Vup(:,:,t);
end

% Time step for the last measurment update
t_last_update = t;

% Iterate backwards in time
for t = t_last_update-1:-1:1
    % Compute smoothed parameters
    [msm(:,t),Psm(:,:,t),vsm(t),Vsm(:,:,t)] = factorizedGIWsmoothing(...
        mup(:,t),Pup(:,:,t),vup(t),Vup(:,:,t),...
        msm(:,t+1),Psm(:,:,t+1),vsm(t+1),Vsm(:,:,t+1),...
        vpred(t+1),Vpred(:,:,t+1),models);
end