clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

addpath("../")
addpath("../error_ellipse");
addpath("../../../matlab_helpers/");

%% parameter swiping
i_exp = 1;
eps = 0.01;
eps_map = 0.7;
speed = 0.01;
nt = 50;
sig0 = 0.01;
sigT = 0.05;
eta = 1e-5;
stop_err = 1e-4;
max_iter = 30;
% cost_sigma = 5.6e4;

min_iter = max_iter;
best_cost_sigma = 5.6e5;

v_eps = 0.005:0.005:0.05;
v_cost_sigma = 5.6e5: 10000: 5.6e6;
v_eps_map = 0.3:0.05:0.7;
v_speed = 0.005:0.005:0.1;
v_eta = 1e-5:1e-5:1e-3;
v_stop_err = [1e-6, 1e-5, 1e-4];

for eps = v_eps(1:end)
    for eps_map = v_eps_map(1:end)
        for speed = v_speed(1:end)
            for eta = v_eta(1:end)
                for stop_err = v_stop_err(1:end)
                    for cost_sigma = v_cost_sigma(1:end)   
     
    args = [num2str(i_exp), ' ', num2str(eps), ' ', num2str(eps_map), ' ', num2str(speed), ' ', num2str(nt), ' ', num2str(sig0), ' ', num2str(sigT), ' ', ...
        num2str(eta), ' ', num2str(stop_err), ' ', num2str(max_iter), ' ', num2str(cost_sigma)];
    
    command = ['/home/hongzhe/git/VIMP/vimp/build/pgcs_PlanarPRModel', ' ', args];
    num_iter = system(command)
    if num_iter < min_iter          
        min_iter = num_iter;
        best_args = [eps, eps_map, speed, nt, sig0, sigT, eta, stop_err, max_iter, cost_sigma];
    end
                    end
                end
            end
        end
    end
end

% run best cost_sig
args = [num2str(i_exp), ' ', num2str(best_args(1)), ' ', num2str(best_args(2)), ' ', num2str(best_args(3)), ' ', num2str(best_args(4)), ...
    ' ', num2str(best_args(5)), ' ', num2str(best_args(6)), ' ', num2str(best_args(7)), ' ', num2str(best_args(8)), ' ', ...
    num2str(best_args(9)), ' ', num2str(best_args(10))];
    
command = ['/home/hongzhe/git/VIMP/vimp/build/pgcs_PlanarPRModel', ' ', args];
num_iter = system(command)

% read map
sdfmap = csvread("../../RAL-examples/2d_pR/map2/map_multiobs_map2.csv");

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')

for i = 1:1 

    nexttile
    hold on
    prefix = ["map2/case"+num2str(i)+"/"];

    % % --- read means and covariances ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    plot_2d_result(sdfmap, means, covs);

    xlim([-20, 25]);
    ylim([-15, 22]);

end



%% read map
sdfmap = csvread("../../RAL-examples/2d_pR/map2/map_multiobs_map2.csv");

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight')

for i = 1:4 % 4 experiments
%     x0 = 500;
%     y0 = 500;
%     width = 1290.427199;
%     height = 800;
%     figure
%     set(gcf,'position',[x0,y0,width,height])
%     tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')

    nexttile
    hold on
    prefix = ["map2/case"+num2str(i)+"/"];
    % % --- read means and covariances ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    plot_2d_result(sdfmap, means, covs);

%     % --- read baselines ---
%     means_prm = csvread([prefix + "prm_5000.csv"]);
%     means_rrt = csvread([prefix + "RRTstar_3000.csv"]);
% 
%     % --- plot baselines ---
%     [~, nt_prm] = size(means_prm);
%     [~, nt_rrt] = size(means_rrt);
%     
%     for i_pt = 1:nt_prm-1
%         plot([means_prm(1, i_pt), means_prm(1, i_pt+1)], ...
%             [means_prm(2, i_pt), means_prm(2, i_pt+1)], 'LineWidth', 3.0, 'Color', 'b');
%     end
%     
%     hold on
% 
%     for i_pt = 1:nt_rrt-1
%         plot([means_rrt(1, i_pt), means_rrt(1, i_pt+1)], ...
%             [means_rrt(2, i_pt), means_rrt(2, i_pt+1)], 'LineWidth', 3.0, 'Color', 'g');
%     end
    
%     if i==3
        xlim([-20, 25]);
        ylim([-15, 22]);
%     end
%     axis off ; 

end
