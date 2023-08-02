%% Gaussian Hermite cubature

%% One dimension
syms x
Kinv = 10000;
gx = x*Kinv*x;
Int = GaussHermitOneDim(gx, 6, 0, 0.0001)

%% Higher dimensions
n = 2;
m = ones(n, 1);
P = eye(n);
p = 10; % Hermite polynomial degree
x = sym('x', [n 1]);

% gx = sum(x.^2);
gx = [3*x1^2; 2*x2*x1];

Integral = GaussHermiteN(n, gx, p, m, P)

%%
% target Gaussian -log_prob
precision_t = [1,-0.74; -0.74,1];
cov_t = inv(precision_t);
mean_t = ones(n, 1);

prob = expm(-(x-mean_t)'*precision_t*(x-mean_t)/2.0) / sqrt((2*pi)^n * det(cov_t));
gx = -log(prob);
% gx = sum(x.^2);

% gx = x1^2 + x2^2;
gx = [3*x1^2; 2*x2*x1];

Integral = GaussHermiteN(n, gx, p, mean_t, cov_t)

%% draft
getWeight(10)


