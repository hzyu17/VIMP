%% inverser for a given sparse matrix.
clc
clear all
m  = csvread("../vimp/tests/precision.csv");
sp_m = sparse(m);

%% dense
[L, D] = ldl(m)

norm(L*D*L' - m, 'fro') < 1e-10
% L: all 1 diags
norm(diag(L)-ones(size(L, 1), 1)) == 0

%% sparse
[L, D, P] = ldl(sp_m);

% verify 
norm(L*D*L' - P'*sp_m*P, 'fro') < 1e-10
norm(P'*P-eye(size(sp_m, 1)), 'fro') < 1e-10
norm(P*L*D*L'*P'-sp_m, 'fro') < 1e-10
% L: all 1 diags
norm(diag(L)-ones(size(L, 1), 1)) == 0

L_new = P*L;
diag(L)
% L_new: all 1 diags
norm(diag(L_new)-ones(size(L_new, 1), 1)) 

norm(L_new*D*L_new' - sp_m, 'fro') < 1e-10

% inverse 
sp_invm = sparse(zeros(size(m)));

