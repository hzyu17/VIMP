clear all
close all
clc

% =================== problem settings =================== 
nx = 6;
nu = 2;

% time scaling      
sig = 2;
nts = [500, 1000, 2000, 3000, 4000];

% sig = 3;
% nts = [500, 1000, 2000, 3000, 4000];

% sig = 3;
% nts = [2000, 3000, 4000, 5000, 6000];

% sig = 6;
% nts = [20000, 30000];


%%
m0 = [4; 4; 0.0; 0; 0; 0];
Sig0  =  0.01 .* eye(6);

[A1, B1, a1] = linearization(m0);

% pinvBBT = [   0         0         0         0         0         0;
%                          0         0         0         0         0         0;
%                          0         0         0         0         0         0;
%                          0         0         0         0         0         0;
%                          0         0         0         0    0.1181   0;
%                          0         0         0         0         0    1.173512e-04];

pinvBBT = [   0         0         0         0         0         0;
                         0         0         0         0         0         0;
                         0         0         0         0         0         0;
                         0         0         0         0         0         0;
                         0         0         0         0        1          0;
                         0         0         0         0         0        1];

[nx, nu] = size(B1);

m1 = [2; 2; 0; 0; 0; 0];
Sig1 = 0.1 .* eye(6);

eta = 1e-6;
epsilon = 1e-4;
stop_err = 1e-5;

% state cost
Q1 = zeros(nx, nx);
lin_err_nt = [];

for nt=nts
% nt = 2000;

    % 3D matrices
    A  = A1(:)*ones(1,nt);
    As  = reshape(A,[nx,nx,nt]);
    B  = B1(:)*ones(1,nt);
    B  = reshape(B,[nx,nu,nt]);
    r  = zeros(nx,nt);
    as = a1(:)*ones(1,nt);
    Q  = Q1(:)*ones(1,nt);
    Q  = reshape(Q,[nx,nx,nt]);
    
    % profile on
    % tic_solving = tic;
    [K,d, As, B, as,zk,Sk] = optimize(nt, As, B, pinvBBT, as, Q, epsilon, m0, Sig0, m1, Sig1, eta, sig, stop_err);
    is_all_psd(Sk)

    [Skstar, zkstar]     = Szk(As,B,as,epsilon,m0,Sig0,sig);
    [hAstar,hastar,nTr] = linearAa_pquad(Skstar,zkstar,As);
    
    lin_err = linearization_error(hAstar, hastar, B, zkstar, nt, sig);
    
    lin_err_nt = [lin_err_nt, norm(lin_err)];

end

figure
grid minor
hold on
title('Linearization error')
plot(nts, lin_err_nt, LineWidth=2.0)


