function integral_GH = GaussHermitOneDim(gx, p, m, P)
% The numerical approximation using Gauss-Hermite quadrature of gx*N(m,P) on [-inf, inf]
% reference: 'Beyesian filtering and smoothing', Simo S¨arkk¨a, p.p. 101 
%   Input:
%               gx: the function to be integrated, the variable should be
%               named as 'x'
%               p: the degree of Hermite polynomial     
%               m: mean of the Gaussian distribution
%               sig: stdv of the Gaussian distribution

%% the roots of p Hermit polynomials (probabilistic sense)
% recurrence relation: H_j+1 = x*H_j - j*H_(j-1): a_j = 1, b_j = 0, c_j = (j-1) 
[roots, W] = getWeight(p);

%% Integrating
syms x
sig = sqrt(P);
% function definitions
gaussian_x = 1/sqrt(2*pi)/sig*exp(-(x-m)^2/P/2);
func = gx*gaussian_x;
func_hd = matlabFunction(func);

% ground truth
integral_true = integral(func_hd, -inf, inf)

% approximation
pts = sig.*roots + m;
integral_GH = sum(W.*double(subs(gx, pts)));

end