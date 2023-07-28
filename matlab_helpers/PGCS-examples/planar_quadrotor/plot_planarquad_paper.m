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
nx = 6; nu = 2; sig = 4.0; epsilon=0.001;

prefix = "map2/case1";
Ks = csvread(prefix+"/Kt_sdf.csv");
ds = csvread(prefix+"/dt_sdf.csv");
zkt = csvread(prefix+"/zk_sdf.csv");
Skt = csvread(prefix+"/Sk_sdf.csv");
hAstar_sdf = csvread(prefix+"/hAkt_sdf.csv");

hastar_sdf = csvread(prefix+"/hakt_sdf.csv");

nt = size(Ks, 2);
hAstar_sdf = reshape(hAstar_sdf, [nx, nx, nt]);
Ks = reshape(Ks, [nu, nx, nt]);
Skt = reshape(Skt, [nx, nx, nt]);

B1 = 10.*[  0, 0; 
            0, 0; 
            0, 0; 
            0, 0; 
            1/sqrt(2), 1/sqrt(2); 
            1/sqrt(2), -1/sqrt(2)];
B = B1(:)*ones(1,nt);
B  = reshape(B,[nx,nu,nt]);
% quadrotor configurations
L = 1;
H = 0.2;

% plot map and nominals
x0 = 50;
y0 = 50;
width = 400;
height = 350;

%% ------------ compute closed-loop linearized systems ----------
dt = sig/(nt-1);
mus = zeros(6, nt);
m0 = zkt(:,1);
mus(:,1) = m0;
mu = m0;
for i=1:nt-1
    Acl = hAstar_sdf(:,:,i) + B(:,:,i)*Ks(:,:,i);
    acl = hastar_sdf(i) + B(:,:,i)*ds(:,i);
    mu = mu + (Acl*mu + acl) .* dt;
    mus(:,i+1) = mu;
end

Sig0 = Skt(:,:,1);
Sigk = Sig0;
Sigs(:,:,1) = Sig0;
for i=1:nt-1
    Acl = hAstar_sdf(:,:,i) + B(:,:,i)*Ks(:,:,i);
    acl = hastar_sdf(i) + B(:,:,i)*ds(:,i);
    Sigk = Sigk + (Acl*Sigk + Sigk*Acl' + epsilon*B(:,:,i)*B(:,:,i)').*dt;
    Sigs(:,:,i+1) = Sigk;
end

%% --------- plot nominals ----------
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
hold on

% plot map and nominals
plot_nominals_with_sdf(zkt, Sigs, Ks, ds, nt, sig, epsilon, sdfmap);

%% ------------- rotation angles -------------
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
