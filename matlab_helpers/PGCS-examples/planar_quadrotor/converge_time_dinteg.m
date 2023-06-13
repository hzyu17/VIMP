%% count the convergence time wrt. the time discretization.

clear all
close all
clc

% =================== problem settings =================== 
nx = 4;
nu = 2;

% time scaling
sig = 5.0;
epsilon = 0.01;

% results 
converge_times = zeros(1, 198);
time_discretizations = zeros(1, 198);

iter = 15;

m0 = [1; 8; 2; 0];
Sig0  =  0.01 .* eye(4);

m1 = [1; 2; -1; 0];
Sig1 = 0.1 .* eye(4);

% randomly initialize A1
% rng(0)
% x0 = rand(4,1);
% [A1, B1, a1] = linearization(x0, sig);

[A1, B1, a1] = linearization(m0, sig);
% state cost
Q1 = 0.1.*eye(nx);

% compute convergence time
index = 1;
profile on
nt = 25
eta = 1e-6;
time_discretizations(index) = nt;
tic_solving = tic;
[K,d] = optimize(nt, A1, B1, a1, Q1, epsilon, m0, Sig0, m1, Sig1, eta, sig, 1e-4);
solving_time = toc(tic_solving)

converge_times(index) = solving_time;
index = index + 1;

profile viewer
%% plotting
x0 = 50;
y0 = 50;
width = 400;
height = 350;
time_discretizations = [25 100 500 1000 2000];
converge_times = [0.098 0.088 0.128 0.192 0.298];
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
hold on 
grid minor
set(gca,'fontsize',16);
plot(time_discretizations, converge_times, 'Linewidth', 2.5)
xlabel('Time discretization','Interpreter','latex')
ylabel('Convergence Time','Interpreter','latex')
xlim([150, 2000])