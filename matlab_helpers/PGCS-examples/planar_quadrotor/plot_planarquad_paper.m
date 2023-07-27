clear all
close all
clc

%% Read optimization results
nx = 6; nu = 2;

prefix = "map2/case1";
Kt = csvread(prefix+"/Kt_sdf.csv");
dt = csvread(prefix+"/dt_sdf.csv");
zkt = csvread(prefix+"/zk_sdf.csv");
Skt = csvread(prefix+"/Sk_sdf.csv");

nt = size(Kt, 2);
Kt = reshape(Kt, [nu, nx, nt]);
Skt = reshape(Skt, [nx, nx, nt]);

%% ========== plot the mean quadrotor trajectory ==========
T = 2.0;
x0 = 50;
y0 = 50;
width = 400;
height = 350;
t  = linspace(0,T,nt);
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
hold on 
step_size = 100;

for ti=1:30:nt
    mean_x = zkt(1, ti);
    mean_y = zkt(2, ti);
    angle_phi = zkt(3, ti) / pi * 180;
    draw_planarquad([mean_x,mean_y], 0.25, 0.05, angle_phi, [0.1,0.1,0.1]);
end