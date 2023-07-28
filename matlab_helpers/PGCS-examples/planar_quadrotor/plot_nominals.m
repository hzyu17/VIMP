function output = plot_nominals(zk, Sk, Ks, ds, nt, sig, epsilon)
%PLOT_NOMINALS Summary of this function goes here
output = 1;
nx = 6;

% ---------------------------- positions ---------------------------- 
step_size = 10;
Sig0 = Sk(1:end, 1:end, 1);
Sig0_xy = Sk(1:2, 1:2, 1);
m0 = zk(1:end, 1);
Sk_xy = Sk(1:2, 1:2, 1:end);
zk_2d = zk(1:2, :);
plot_marginal(m0, zk_2d, Sig0_xy, Sk_xy, nt, sig, step_size);
set(gca,'fontsize',16);
xlabel('Time $t$','Interpreter','latex'),ylabel('Position $x$','Interpreter','latex');
zlabel('Position $y$','Interpreter','latex');


% ---------------------------- sampled trajectories ----------------------------
n2=6;

t  = linspace(0,sig,nt);
cstring='grbcmk';
for j=1:n2
    init = randn(6,1);
    init = Sig0^(1/2)*init+m0;
    [~,x,u] = planar_quadrotor(Ks,ds,epsilon,init,sig);

    plot3(t,x(1,:),x(2,:),cstring(mod(j,6)+1),'LineWidth',2);
end

% plot the mean quadrotor trajectory
x0 = 50;
y0 = 50;
width = 400;
height = 350;
t  = linspace(0,sig,nt);
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
hold on 
% step_size = 100;

for ti=1:step_size:nt
    mean_x = zk(1, ti);
    mean_y = zk(2, ti);
    angle_phi = zk(3, ti) / pi * 180;
    draw_planarquad([mean_x,mean_y], 0.25, 0.05, angle_phi, [0.1,0.1,0.1]);
end

%
% ---------------------------- velocities ---------------------------- 
x0 = 50;
y0 = 50;
width = 400;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
hold on 

Sig0_vxy = Sig0(4:5, 4:5, 1:end);
Sk_vxy = Sk(4:5, 4:5, 1:end);
vzk_2d = zk(4:5, :);
v0 = m0(4:5);
plot_marginal(v0, vzk_2d, Sig0_vxy, Sk_vxy, nt, sig, nt);
set(gca,'fontsize',16);
xlabel('Time $t$','Interpreter','latex'),ylabel('Velocity $v_x$','Interpreter','latex');
zlabel('Velocity $v_y$','Interpreter','latex');
% xlim([0,5])

% ---------------------------- sampled trajectories ----------------------------
cstring='grbcmk';
for j=1:n2
    init = randn(nx,1);
    init = Sig0^(1/2)*init+m0;
    [~,x,u] = planar_quadrotor(Ks,ds,epsilon,init,sig);
    plot3(t,x(4,:),x(5,:),cstring(mod(j,6)+1),'LineWidth',2);
end

% =================== trajectories and control inputs =================== 
x0 = 50;
y0 = 50;
width = 400;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
hold on
cstring='grbcmk';
for j=1:n2
    init = randn(nx,1);
    init = Sig0^(1/2)*init+m0;
    [~,x,u] = planar_quadrotor(Ks,ds,epsilon,init,sig);
    plot(t,u(1,:),cstring(mod(j,6)+1),'LineWidth',2);
end
hold off;

xlabel('Time $t$','Interpreter','latex'),ylabel('Control input $u$','Interpreter','latex');
set(gca,'fontsize',16);
end

