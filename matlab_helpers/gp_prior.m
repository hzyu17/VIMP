%% Gaussian process priors
% Author: Hongzhe Yu
% date: 07/31/2022

%% GP linear
dim = 2;
Qc = eye(2);
dt = 0.01;
Phi = [eye(dim), dt.*eye(dim);
            zeros(dim, dim), eye(dim)]

Q = [dt^3/3.*Qc, dt^2/2.*Qc;
        dt^2/2.*Qc, dt.*Qc];

x1 = [1.5; 1.0; 0.1; 0.1];
x2 = [1.1; 1.2; 0.3; 0.2];

cost = (Phi*x1 - x2)'*inv(Q)*(Phi*x1 - x2)/2

%% Fixed GP prior
K = eye(2);
mu = ones(2, 1);
x = [1.2; 2.1];
cost = (x-mu)'*inv(K)*(x-mu)

%% Qc_inv
syms qc t real
Q = [t^3/3.*qc, t^2/2.*qc;
        t^2/2.*qc, t.*qc]
qc = 1
t = 0.01
invQ = inv(Q)

