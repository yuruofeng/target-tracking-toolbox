function [E_logdetX,E_X,E_invX] = approximateMatrixTransformationExpectedValues(F,P,Mm,M1,M2,I_x,d)

% Number of states involved in transformation matrix
nIx = length(I_x);

% Evaluate Vx=MxVMx', and its first two derivatives
Vm = Mm*F*Mm.';
invVm = Vm\eye(d);
Vp = zeros(d,d,nIx);
Vb = zeros(d,d,nIx,nIx);
for i=1:nIx
    Vp(:,:,i) = M1(:,:,i)*F*Mm.'+Mm*F*M1(:,:,i).';
    for j=1:nIx
        Vb(:,:,i,j) = M2(:,:,i,j)*F*Mm.'...
            +M1(:,:,j)*F*M1(:,:,i).'...
            +M1(:,:,i)*F*M1(:,:,j).'...
            +Mm*F*M2(:,:,i,j).';
    end
end

% Ensure symmetry
invVm = 0.5*(invVm+invVm.');

% Evaluate log(det(Vx)), and its first two derivatives
% Evaluate Vx^-1, and its first two derivatives
lVm = log(det(Mm*F*Mm.'));
lVp = zeros(1,1,nIx);
lVb = zeros(1,1,nIx,nIx);
invVb = zeros(d,d,nIx,nIx);
for i=1:nIx
    lVp(:,:,i) = trace(invVm*Vp(:,:,i));
    for j=1:nIx
        lVb(:,:,i,j) = trace(-invVm*Vp(:,:,j)*invVm*Vp(:,:,i)...
            +invVm*Vb(:,:,i,j));
        invVb(:,:,i,j) = invVm*(Vp(:,:,j)*invVm*Vp(:,:,i) ...
            -Vb(:,:,i,j) ...
            +Vp(:,:,i)*invVm*Vp(:,:,j))*invVm;
    end
end

% Compute Taylor expansion of Vx and of log(det(Vx))
E_logdetX = lVm;
E_X = Vm;
E_invX = invVm;
for i=1:nIx
    for j=1:nIx
        E_logdetX = E_logdetX+0.5*lVb(:,:,i,j)*P(I_x(i),I_x(j));
        E_X = E_X + 0.5*Vb(:,:,i,j)*P(I_x(i),I_x(j));
        E_invX = E_invX + 0.5*invVb(:,:,i,j)*P(I_x(i),I_x(j));
    end
end

% Ensure symmetry
E_X = 0.5*(E_X+E_X.');
E_invX = 0.5*(E_invX+E_invX.');