clc
%% Verification code for one dimensional case
mu_p = 20;
sig_p_sq = 9;
f = 400;
b = 0.1;
sig_r_sq = 0.09;
y = mu_p+0.05;

syms x real
gx1 = x;
gx2 = x^2;
gx3 = x^3;
gx4 = x^4;

mu = mu_p;

GaussHermitOneDim(gx1, p, mu, P);
GaussHermitOneDim(gx2, p, mu, P);
GaussHermitOneDim(gx3, p, mu, P);
GaussHermitOneDim(gx4, p, mu, P);

phi = (x-mu_p)^2/sig_p_sq/2 + (y-f*b/x)^2/sig_r_sq/2;
phi_func = matlabFunction(phi);

xmu_phi = (x-mu).*phi;
xmu_phi_func = matlabFunction(xmu_phi);

xmumuT_phi = (x-mu)*(x-mu).*phi;
xmumuT_phi_func = matlabFunction(xmumuT_phi);


%% function values


%% Gauss Hermite
p = 6;
mu = mu_p;
P = 9;
disp('E_Phi')
Int1 = GaussHermitOneDim(phi, p, mu, P);
disp('E_xmu_phi')
Int2 = GaussHermitOneDim(xmu_phi, p, mu, P);
disp('E_xmumuT_phi')
Int3 = GaussHermitOneDim(xmumuT_phi, p, mu, P);
