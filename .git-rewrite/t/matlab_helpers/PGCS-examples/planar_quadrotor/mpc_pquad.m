clear all
close all
clc

% =================== problem settings =================== 
nx = 6;
nu = 2;

% time scaling
sig = 2.0;

%%
% m0 = [5; 8; 0.0; 0; 0; 0];
m0 = [4; 4; 0.0; -0.2; -0.2; 0];
Sig0  =  0.01 .* eye(6);

[A1, B1, a1] = linearization(m0);

m1 = [2; 2; 0; 0; 0; 0];
Sig1 = 0.05 .* eye(6);

eta = 1e-6;
epsilon = 1e-4;
stop_err = 1e-4;

% state cost
Q1 = zeros(nx, nx);
% Q1(4:6,4:6) = 1e1.*eye(3);

nt = 2000;

% 3D matrices
A  = A1(:)*ones(1,nt);
As  = reshape(A,[nx,nx,nt]);
B  = B1(:)*ones(1,nt);
B  = reshape(B,[nx,nu,nt]);
r  = zeros(nx,nt);
as = a1(:)*ones(1,nt);
Q  = Q1(:)*ones(1,nt);
Q  = reshape(Q,[nx,nx,nt]);

% mpc params
m_cur = m0;
Sig_cur = Sig0;
% sig_cur = sig;

x_nl = zeros(6, nt);
x_nl(1:end, 1) = m0;

for i_mpc = 1:nt-1
    %% optimize
    if i_mpc == 1
        [K,d, As, B, as,zk,Sk] = optimize(nt, As, B, as, Q, epsilon, m_cur, Sig0, m1, Sig1, eta, sig, stop_err);
    else
        [K,d, As, B, as,zk,Sk] = optimize(nt, hAstar, B, hastar, Q, epsilon, m_cur, Sig0, m1, Sig1, eta, sig, stop_err);
    end

    %% recover control
    [Skstar, zkstar]     = Szk(As,B,as,epsilon,m_cur,Sig_cur,sig);
    [hAstar,hastar,nTr] = linearAa_pquad(Skstar,zkstar,As);
    % linearization_error(hAstar, hastar, B, zkstar, nt, sig);
    Qstar = Q;
    rstar= zeros(nx, nt);
    for i = 1:nt
        rstar(:, i) = (-2*Q(:,:,i)*zkstar(:, i) + nTr(:, i))/2;
    end
    
    [Ks,ds] = linearCov(hAstar,hastar,B,epsilon,Qstar,rstar,m_cur,Sig_cur,m1,Sig1,sig);
    
    K1 = Ks(:,:,1:2);
    d1 = ds(:,1:2);
    %% execute
    [~,x,u] = planar_quadrotor(K1,d1,epsilon,m_cur,sig);
    m_cur = x(:, 2);
    x_nl(:, i_mpc+1) = m_cur;
    
end

