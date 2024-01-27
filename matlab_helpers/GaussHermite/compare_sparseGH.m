addpath('SparseGH')

% =============== integrand: some function that evaluates g(x): (R times D)->(R times 1)
mu_p = 20;
sig_p_sq = 9;

f = 400;
b = 0.1;
sig_r_sq = 0.09;
y = f*b/mu_p + 0.05;

syms x real

phi = (x-mu_p)^2/sig_p_sq/2 + (y-f*b/x)^2/sig_r_sq/2;
phi_func = matlabFunction(phi, 'Vars', x);

xmu_phi = (x-mu).*phi;
xmu_phi_func = matlabFunction(xmu_phi, 'Vars', x);

xmumuT_phi = (x-mu)*(x-mu).*phi;
xmumuT_phi_func = matlabFunction(xmumuT_phi, 'Vars', x);


%% ===== Gauss Hermite =====
% Weights
p = 10;
[roots, W] = getWeight(p)

%% Naive GH implementation for the integrations
p = 6;
mu = mu_p;
P = 9; %covariance, =sig^2
sig = sqrt(P);

%% Sparse GH integration
D = 1;    % dimensions
maxk = 6;  % max. accuracy level (pol. exactness wil be 2k-1)

% sparse GH sigma points and weights
[x_sp w_sp] = nwspgr('GQN', D, maxk);

% ===== Shift the mean and cov of the sigma points ===== 
pts = sig.*x_sp + mu;

% ------------- integrate function \phi(x) -------------
% naive full GH result 
disp('E_Phi')
Int1 = GaussHermitOneDim(phi, p, mu, P)

% sparse GH result
g1 = phi_func(pts);
SGappr1 = g1'*w_sp

% ------------- integrate function (x-\mu)*\phi(x) -------------
disp('E_xmu_phi')
Int2 = GaussHermitOneDim(xmu_phi, p, mu, P)

% sparse GH result
g2 = xmu_phi_func(pts);
SGappr2 = g2'*w_sp

% ------------- integrate function (x-\mu)(x-\mu)^T*\phi(x) -------------
disp('E_xmumuT_phi')
Int3 = GaussHermitOneDim(xmumuT_phi, p, mu, P)

% sparse GH result
g3 = xmumuT_phi_func(pts);
SGappr3 = g3'*w_sp