function GGIW=SmootherAdapter(Bern,modelPMBM)
% Process noise
    T=1;
    sigma=0.1;
    models.n_x = 5;
    Q = sigma^2*[
                T^4/4   0       T^3/2   0     0;
                0       T^4/4   0       T^3/2 0;
                T^3/2   0       T^2     0     0;
                0       T^3/2   0       T^2   0;
                0       0       0       0     1e-8];
    % Process noise parameters for extent
    n_IW = inf;
    % Measurement noise
    sig_r = 0.1;
    R = diag([sig_r sig_r].^2);

    % Target extent, aligned to x-axis. Extent is assumed to be aligned with
    % the heading of the simulated target. The extent corresponds to Ns
    % standard deviations
    % Factorized models
% Process model
    models.motionModel = @LinearVelocityConstantTurnrate;
    models.matrixTransformationFunction = @RotationMatrixLVCT;
    models.inverseMatrixTransformationFunction = @inverseRotationMatrixLVCT;
    models.Q = Q;
    models.Ts = T;
    models.d = 2;
    models.n = n_IW;
    models.updateType = '';
    models.H = [eye(models.d) zeros(models.d,models.n_x-models.d)];
    models.Id = eye(models.d);
    models.R = R;
    models.scaleFactor = 1;

    GIWfilters = cell(1,0);
    GIWfilters{1} = factGIW_forwardFilter_backwardSmoother;
    GIWfilters{1}.models = models;
    GIWfilters{1}.models.KLdiv_minimization_flag = 0;
    
    for i=1:length(Bern.GGIW)
        %m
        Bern.GGIW(i).m(5)=0;
        Bern.GGIWpre(i).m(5)=0;
        GIWfilters{1}.mpred(1:5,i)=Bern.GGIWpre(i).m;
        GIWfilters{1}.mup(1:5,i)=Bern.GGIW(i).m;
        %P
        Bern.GGIWpre(i).P(5,1:4)=1e-8;
        Bern.GGIWpre(i).P(1:4,5)=1e-8;
        Bern.GGIW(i).P(5,1:5)=1e-8;
        Bern.GGIW(i).P(1:5,5)=1e-8;
        GIWfilters{1}.Ppred(:,:,i)=Bern.GGIWpre(i).P;
        GIWfilters{1}.Pup(:,:,i)=Bern.GGIW(i).P;
        %v
        GIWfilters{1}.vpred(i)=Bern.GGIWpre(i).v;
        GIWfilters{1}.vup(i)=Bern.GGIW(i).v;
        %V
        GIWfilters{1}.Vpred(:,:,i)=Bern.GGIWpre(i).V;
        GIWfilters{1}.Vup(:,:,i)=Bern.GGIW(i).V;
    end
    GIWfilters{1}.backwardSmoother;
    for i=1:length(Bern.GGIW)
        %m
        GGIW(i).m=GIWfilters{1}.msm(1:4,i);
        %P
        GGIW(i).P=GIWfilters{1}.Psm(1:4,1:4,i);
        %v
        GGIW(i).v=GIWfilters{1}.vsm(i);
        %V
        GGIW(i).V=GIWfilters{1}.Vsm(:,:,i);
    end
end
