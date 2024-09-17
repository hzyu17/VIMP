clear all
close all
clc
%% read zk_star and plot

zk_sdf = csvread("/home/hongzhe/git/VIMP/vimp/tests/zk_sdf.csv");
Sk_sdf = csvread("/home/hongzhe/git/VIMP/vimp/tests/Sk_sdf.csv");
Kt_sdf = csvread("/home/hongzhe/git/VIMP/vimp/tests/Kt_sdf.csv");
dt_sdf = csvread("/home/hongzhe/git/VIMP/vimp/tests/dt_sdf.csv");

nt = size(zk_sdf, 2);
zk_pos = zk_sdf(1:2, :);
Sk_sdf = reshape(Sk_sdf, 4,4,nt);

figure 
grid on
hold on
for i=1:nt
    scatter(zk_pos(1, i), zk_pos(2, i));
end
xlim([-20, 20])
ylim([-10, 20])

% %% plot marginal function
% sig = 5.0;
% m0 = [-10, -5, 2, 0]';
% % plot marginals
% Sig0 = 0.01*eye(4);
% step_size = floor(nt/5);
% Sig0_xy = Sig0(1:2, 1:2);
% Sk_xy = Sk_sdf(1:2, 1:2, 1:end-2);
% zk_2d = zk_sdf(1:2, 1:end-2);
% plot_marginal(m0, zk_2d, Sig0_xy, Sk_xy, nt, sig, step_size);
% set(gca,'fontsize',16);
% xlabel('Time $t$','Interpreter','latex'),ylabel('Position $x$','Interpreter','latex');
% zlabel('Position $y$','Interpreter','latex');
% 
% xlim([0, 5])
% ylim([-20, 20])
% zlim([-10, 20])
