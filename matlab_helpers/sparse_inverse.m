%% inverser for a given sparse matrix.
clc
clear all

m  = csvread("../vimp/data/2d_pR/map_narrow/circumvent/precisoin.csv");
size2 = size(m, 2);
ii = 5;
m = m(size2*(ii-1)+1 : size2*ii, 1 : size2);
sp_m = sparse(m);

%% dense
% [L, D] = ldl(m);
% 
% norm(L*D*L' - m, 'fro') < 1e-10;
% % L: all 1 diags
% norm(diag(L)-ones(size(L, 1), 1)) == 0;

% %% ldlt
% % ------------------------ with P matrix ------------------------ 
% [L, D, P] = ldl(sp_m);
% L_new = P*L;
% L_new_f = full(L_new);
% norm(L_new*D*L_new' - sp_m, 'fro') < 1e-10;
% % inverse 
% sp_invm = sparse(zeros(size(m)));

%% ------------------------ ldl without P matrix ------------------------ 
[L, D] = ldl(m);
L_sp = sparse(L);
D_sp = sparse(D);

% --- plotting --- 
x0 = 500;
y0 = 500;
width = 600;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'tight')
nexttile
title("\Sigma^{-1}", "Interpreter","tex")
hold on
spy(m)
nexttile
title("L", "Interpreter","tex")
hold on
spy(L)

%% The inverse algorithm
T = size(m, 1);
covariance = inv(m);

profile on

disp('=============== algorithm inverse ===============')
tic
cov = zeros(size(m));
% cov = sparse(cov);

dim_conf = 2;
dim_theta = 2*dim_conf;
dim_nzz = 2*dim_theta;

cov(T, T) = 1/D(T, T);
% L = sparse(L);

for j=T-1 : -1 : 1 % col
    for i = min(j+1+dim_nzz, T): -1 : j
        val = 0;
        if i == j
            val = 1/D(i,i);
        end
        for l = j+1 : min(j+1+dim_nzz, T)
            val = val - cov(i, l)*L(l, j);
        end
        cov(i, j) = val;
        if i ~= j
            cov(j, i) = val;
        end
    end
end
toc

disp('=============== full inverse ===============')
tic
covariance = inv(m);
toc

profile viewer

%%
cov_f = full(cov);

figure
spy(cov)

P1 = [eye(4), zeros(4, 56)];
diff1 = norm(P1*covariance*P1' - P1*cov_f*P1', 'fro')

k = 7;
P2 = [zeros(8, 4+8*(k-1)), eye(8,8), zeros(8, 60-4-8*k)];
figure
spy(P2)
diff2 = norm(P2*covariance*P2' - P2*cov_f*P2', 'fro')

P3 = [zeros(4, 56), eye(4)];
diff3 = norm(P3*covariance*P3' - P3*cov_f*P3', 'fro')

%% C++ code debug
m  = csvread("/home/hongzhe/git/VIMP/vimp/data/2d_pR/map_narrow/circumvent/precisoin.csv");
% csvwrite("/home/hongzhe/git/VIMP/vimp/data/2d_pR/map_narrow/circumvent/precisoin.csv", m);

size2 = size(m, 2);
ii = 5;
m = m(size2*(ii-1)+1 : size2*ii, 1 : size2);

covariance = inv(m);

csvwrite("/home/hongzhe/git/VIMP/vimp/test_sp_inverse_prec.csv", m);

% L_ = csvread("../vimp/L_.csv");
% D_ = csvread("../vimp/D_.csv");

cov_c = csvread("../vimp/test_sp_inverse_cov.csv");

P1 = [eye(4), zeros(4, 56)];
diff1 = norm(P1*covariance*P1' - P1*cov_c*P1', 'fro')

k = 7;
P2 = [zeros(8, 4+8*(k-1)), eye(8,8), zeros(8, 60-4-8*k)];
% figure
% spy(P2)
diff2 = norm(P2*covariance*P2' - P2*cov_c*P2', 'fro')

P3 = [zeros(4, 56), eye(4)];
diff3 = norm(P3*covariance*P3' - P3*cov_c*P3', 'fro')
