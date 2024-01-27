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

%% ========= Compare the GH integration for N dimensional functions =========
% Test GaussHermit for a quadratic function
p_GH = 10;

% ============== 2 dim input x = [x_1; x_2], 1 dim output ============== 
% Cost fn
x = sym('x', [1,2]);
prec_cost = 10000.*eye(2);
phi_21 = x * prec_cost * transpose(x)

phi_func_21 = matlabFunction(phi_21, 'Vars', x);

% Inputs
m = ones(2,1);
prec = [1, -0.74; -0.74, 1.0];
cov = inv(prec);
sig = sqrtm(cov);

Int21 = GaussHermiteN(2, phi_21, p_GH, m, cov)

% ------ sparse GH ------
D = 2;    % dimensions
maxk = 6;  % max. accuracy level (pol. exactness wil be 2k-1)

% sparse GH sigma points and weights
[x_sp, w_sp] = nwspgr('GQN', D, maxk);

% ------ Shift the mean and cov of the sigma points ------
pts = x_sp*sig' + m';

g21 = arrayfun(phi_func_21, pts(:, 1), pts(:, 2));
SGappr21 = g21'*w_sp

% ============== 2 dim input x = [x_1; x_2], 2 dim output ============== 
x = sym('x', [1,2]);
phi_22 = [3*x(1)*x(1); 2*x(1)*x(2)]
phi_func_22_1 = matlabFunction(phi_22(1), 'Vars', x);
phi_func_22_2 = matlabFunction(phi_22(2), 'Vars', x);

% Inputs
m = ones(2,1);
prec = [1, -0.74; -0.74, 1.0];
cov = inv(prec);
sig = sqrtm(cov);

% ------ naive GH ------
tic
Int22 = GaussHermiteN(2, phi_22, p_GH, m, cov)
toc

% ------ sparse GH ------
D = 2;    % dimensions
maxk = 6;  % max. accuracy level (pol. exactness wil be 2k-1)

% sparse GH sigma points and weights
tic
[x_sp, w_sp] = nwspgr('GQN', D, maxk);

% ------ Shift the mean and cov of the sigma points ------
num_pts = size(pts, 1)
pts = x_sp*sig' + m';
g21 = zeros(num_pts,2);
SGappr21 = zeros(2, 1);

g21(:,1) = arrayfun(phi_func_22_1, pts(:, 1), pts(:, 2));
g21(:,2) = arrayfun(phi_func_22_2, pts(:, 1), pts(:, 2));

SGappr21 = g21'*w_sp
toc