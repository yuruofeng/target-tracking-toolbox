function [mkT,PkT,vkT,VkT] = factorizedGIWsmoothing(mkk,Pkk,vkk,Vkk,mk1T,Pk1T,vk1T,Vk1T,vk1k,Vk1k,models)

% Models
Q = models.Q;
Ts = models.Ts;
d = models.d;
n = models.n;

% Motion model
[fm,Fm] = models.motionModel(mkk,Ts);

%% Gaussian

% Predicted mean
mk1k = fm;
% Predicted covariance
Pk1k = Fm*Pkk*Fm.' + Q;
% Make sure P is symmetric
Pk1k = 0.5*(Pk1k + Pk1k.');
% Smoothing gain
G = Pkk*Fm.'/Pk1k;

% Smoothed mean
mkT = mkk + G*(mk1T - mk1k);
% Smoothed covariance
PkT = Pkk - G*(Pk1k-Pk1T)*G.';
% Make sure P is symmetric
PkT = 0.5*(PkT + PkT.');

%% Inverse Wishart

% Division of IW k+1|T and IW k+1|k is porportional to IW(e,E)
e = vk1T-vk1k;
E = Vk1T-Vk1k;

% Make sure that n is large enough for the given e
if n<max([2*(d+1)^2/e+0.1 , 3*(d+1)-e+0.1])
    n = max([2*(d+1)^2/e+0.1 , 3*(d+1)-e+0.1]);
end

% inverse transformation matrix
[Mm,M1,M2,I_x] = models.inverseMatrixTransformationFunction(mkT);

if isempty(I_x)
    % No matrix transformation dependent on the kinematic state
    % i.e., M(x) is assumed to be an identity matrix
    
    % Density approximations achieved by matching E[X] and E[X^-1] instead
    % of minimising the KL-div
    k = (e- (2*(d+1)^2)/n )/(1 +e/n - 3*(d+1)/n);
    K = E/(1+(e-3*(d+1))/n);

elseif models.KLdiv_minimization_flag
    % Matrix transformation function is used, density approximations by
    % minimizing the KL divergence
    
    % GB2(Xk ; a, b, A, zeros) can be approximated as IW(Xk ; g, G)  by
    % matching E[X] and E[X^-1]
    g = ( e-2*(d+1)^2/n ) / ( e/n+1-3*d/n-3/n );
    % G = ((1+e/n-3*d/n-3/n)^-1)*iM*E*iM.';
    
    % The matrix density iM(x)*E*iM(x).' induced by N(x;m,P) is approximated as
    % Wishart
    
    % The matrix E must have eigenvalues>0
    [w,v]=eig(E,'vector');
    E = w*diag(max(v,1e-5))*w.';
    % Compute approximations of expected values
    [E_logdetX,E_X,E_invX] = approximateMatrixTransformationExpectedValues(E,PkT,Mm,M1,M2,I_x,d);
    % Wishart approximation
    [h,H] = WishartApproximation(E_X,E_logdetX,g,d);
    
    if h<=d+1
        % If h is too small, there might be numerical issues
        
        % Approximate instead by matching E[X] and E[X^-1]
        E_X_invX = E_X*E_invX;
        E_X_invX_I = E_X_invX - eye(d);
        % Least squares solution
        h = 0.5*(trace(E_X_invX_I\((d+1)*E_X_invX))); %E_X_invX_I(:)\((d+1)*E_X_invX(:));
        H = E_X/h;
    end
    
    % The integral
    % int IW(Xk ; g, G) N(xk ; m, P) dxk
    % can then be approximated by
    % int IW(Xk ; g, ((1+e/n-3*d/n-3/n)^-1)*Yk)W(Yk; h, H) dYk
    % = GB2(Xk ; a, b, A, zeros)
    a = h/2;
    b = (g-d-1)/2;
    A = ((1+e/n-3*d/n-3/n)^-1)*H;
    
    if g>2*d+1 && h>d+1
        
        % GB2(Xk ; a, b , A, zeros) is approximated as IW(Xk ; k, K) by
        % minimising the KL-divergence
        ExpVal_invX = (2*b/(2*a-d-1))*(eye(d)/A);
        ExpVal_logdetX = log(det(A)) + sum(psi(0,eps+a-((1:d)-1)/2)-psi(0,b-((1:d)-1)/2));
        [k,K] = InverseWishartApproximation(ExpVal_invX,ExpVal_logdetX,a,d);
        
    elseif (h+d+1)*g-2*(d+1)^2>0 && h+g-2*(d+1)>0 && h>d+1
        % GB2(Xk ; a, b , A, zeros) is approximated as IW(Xk ; k, K) by
        % matching E[X] and E[X^-1]
        k = ( (h+d+1)*g-2*(d+1)^2 )/( h+g-2*(d+1) );
        K = H*h*(h-d-1)/(h+g-2*d-2);
    else
        k = h;
        K = H;
    end
    
else
    % Matrix transformation function is used, density approximations by
    % matching expected values instead
    
    % GB2(Xk ; a, b, A, zeros) can be approximated as IW(Xk ; g, G)  by
    % matching E[X] and E[X^-1]
    g = ( e-2*(d+1)^2/n ) / ( e/n+1-3*d/n-3/n );
    % G = ((1+e/n-3*d/n-3/n)^-1)*iM*E*iM.';
    
    % The matrix density iM(x)*E*iM(x).' induced by N(x;m,P) is approximated as
    % Wishart
    
    % The matrix E must have eigenvalues>0
    [w,v]=eig(E,'vector');
    E = w*diag(max(v,1e-5))*w.';
    % Compute approximations of expected values
    [~,E_X,E_invX] = approximateMatrixTransformationExpectedValues(E,PkT,Mm,M1,M2,I_x,d);
    % Wishart approximation by matching E[X] and E[X^-1]
    E_X_invX = E_X*E_invX;
    E_X_invX_I = E_X_invX - eye(d);
    % Least squares solution
    h = ((d+1)/d)*trace(E_X_invX_I\E_X_invX);
    
    % Make sure that h is large enough for the given g
    if h+d+1 < max([2*(d+1)^2/g+1 , 3*(d+1)-g+1])
        h = max([2*(d+1)^2/g+1 , 3*(d+1)-g+1])-d-1;
    end
    H = E_X/h;
    
    if (h+d+1)*g-2*(d+1)^2>0 && h+g-2*(d+1)>0 && h>d+1
        % GB2(Xk ; a, b , A, zeros) is approximated as IW(Xk ; k, K) by
        % matching E[X] and E[X^-1]
        k = ( (h+d+1)*g-2*(d+1)^2 )/( h+g-2*(d+1) );
        K = H*h*(h-d-1)/(h+g-2*d-2);
    else
        k = 0;
        K = 0;
    end
    
end

% IW(Xk ; vkk, Vkk) IW(Xk ; k, K) is proportional to IW(Xk, vkk+k , Vkk+K)
vkT = vkk + k;
VkT = Vkk + K;

% Make sure V is symmetric
VkT = 0.5*(VkT + VkT.');

end



