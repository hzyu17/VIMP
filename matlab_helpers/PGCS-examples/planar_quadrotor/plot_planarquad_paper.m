clear all
close all
clc

addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

addpath("../")
addpath("../error_ellipse");
addpath("../../../matlab_helpers/");

%% read map
sdfmap = csvread("../../../vimp/data/vimp/2d_pR/map_multiobs_map2.csv");

%% Read optimization results
nx = 6; nu = 2; sig = 4.0; epsilon=0.01;

prefix = "map2/case1";
Kt = csvread(prefix+"/Kt_sdf.csv");
dt = csvread(prefix+"/dt_sdf.csv");
zkt = csvread(prefix+"/zk_sdf.csv");
Skt = csvread(prefix+"/Sk_sdf.csv");

nt = size(Kt, 2);
Kt = reshape(Kt, [nu, nx, nt]);
Skt = reshape(Skt, [nx, nx, nt]);

% quadrotor configurations
L = 1;
H = 0.2;

% plot map and nominals
x0 = 50;
y0 = 50;
width = 400;
height = 350;

%% ---------------------------- plot nominals ---------------------------- 
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
hold on

% plot map
cell_size = 0.1;
origin_x = -20;
origin_y = -10;
plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);
plot_nominals(zkt, Skt, Kt, dt, nt, sig, epsilon);

%% ---------------------------- rotation angles ---------------------------- 
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
hold on
grid minor

step_size = 5;
plot_length = floor(nt / step_size);
phi0 = zkt(3, 1);
Sig0  =  Skt(3, 3, 1);
Sk_phi = Skt(3, 3, 1:step_size:end);
Sk_phi = reshape(Sk_phi, [1, plot_length]);
zk_1d = zkt(3, 1:step_size:end);

errorbar(linspace(1,plot_length,plot_length), zk_1d, Sk_phi, 'Linewidth', 1.5)

set(gca,'fontsize',16);
xlabel('Time $t$','Interpreter','latex'),ylabel('Angle $\phi$','Interpreter','latex');

%% ========== plot the mean quadrotor trajectory ==========
T = 4.0;
t  = linspace(0,T,nt);
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
hold on 
step_size = 10;

% plot map
cell_size = 0.1;
origin_x = -20;
origin_y = -10;
plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);
% plot quadrotor positions and orientations
for ti=1:step_size:nt
    mean_x = zkt(1, ti);
    mean_y = zkt(2, ti);
    angle_phi = zkt(3, ti) / pi * 180;
    draw_planarquad([mean_x,mean_y], L, H, angle_phi, [0.1,0.1,0.1]);
end