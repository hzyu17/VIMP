%% Gaussian Hermite cubature

%% One dimension
syms x
gx = x^2;
Int = GaussHermitOneDim(gx, 6, 1, 1.2)

%% Higher dimensions
n = 2;
m = ones(n, 1);
P = eye(n);
p = 10; % Hermite polynomial degree
x = sym('x', [n 1]);

% target Gaussian -log_prob
precision_t = [1,-0.74; -0.74,1];
cov_t = inv(precision_t);
mean_t = zeros(n, 1);

prob = exp(-(x-mean_t)'*precision_t*(x-mean_t)/2.0) / sqrt((2*pi)^n * det(cov_t));
gx = -log(prob);
% gx = sum(x.^2);

Integral = GaussianHermiteN(n, gx, p, [0.773339; 0.73762], inv(PP))

%% draft
getWeight(10)


