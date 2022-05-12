%% Gaussian Hermite cubature

%% One dimension
syms x
gx = x^2;
Int = GaussHermitOneDim(gx, 6, 1, 1.2)

%% Higher dimensions
n = 3;
m = ones(n, 1);
P = 1.2.*eye(n);
p = 10; % Hermite polynomial degree
x = sym('x', [n 1]);
gx = sum(x.^2);

Integral = GaussianHermiteN(n, gx, p, m, P)

%% draft
getWeight(10)


