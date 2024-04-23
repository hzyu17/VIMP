clear all
close all
clc
addpath('../../tools/gtsam_toolbox')
import gtsam.*
import gpmp2.*

addpath("../")
addpath("../../tools/error_ellipse");
addpath("../../");

% read map
sdfmap = csvread("../../../vimp/maps/2dpR/map2/map_multiobs_map2.csv");


%% run 1 experiment
close all
i_exp = 1;
eps = 0.01;
eps_map = 0.7;
speed = 0.0015;
nt = 50;
sig0 = 0.01;
sigT = 0.05;
eta = 5e-6;
stop_err = 1e-5;
max_iter = 50;
cost_sigma = 8e4;

% v_cost_sigma = 2e4: 5e3: 9e4;
% for cost_sigma = v_cost_sigma(1:end)   
    args = [num2str(i_exp), ' ', num2str(eps), ' ', num2str(eps_map), ' ', num2str(speed), ' ', num2str(nt), ' ', num2str(sig0), ' ', num2str(sigT), ' ', ...
            num2str(eta), ' ', num2str(stop_err), ' ', num2str(max_iter), ' ', num2str(cost_sigma)];
        
    command = ['/home/hongzhe/git/VIMP/vimp/build/pgcs_PlanarPRModel', ' ', args];
    num_iter = system(command)
    
     % ------------------ save figure ----------------------
    sdfmap_file = "../../RAL-examples/2d_pR/map2/map_multiobs_map2.csv";
    img_name = ['data/', num2str(i_exp), '_', num2str(eps), '_', num2str(eps_map), '_', num2str(speed), '_', num2str(nt), '_', num2str(sig0), '_', num2str(sigT), '_', ...
        num2str(eta), '_', num2str(stop_err), '_', num2str(max_iter), '_', num2str(cost_sigma)];
    plotting_one_experiment(sdfmap_file, i_exp, img_name, 'on');
    
% end
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

v_eps = 0.005:0.005:0.2;
v_cost_sigma = 1e6: 1e5: 8e6;
v_eps_map = 0.7;
v_speed = 0.005:0.005:0.2;
v_eta = 1e-5:1e-5:1e-3;
v_stop_err = 1e-5;

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
    % ------------------ save figure ----------------------
    sdfmap_file = "../../RAL-examples/2d_pR/map2/map_multiobs_map2.csv";
    img_name = ['data/', num2str(i_exp), '_', num2str(eps), '_', num2str(eps_map), '_', num2str(speed), '_', num2str(nt), '_', num2str(sig0), '_', num2str(sigT), '_', ...
        num2str(eta), '_', num2str(stop_err), '_', num2str(max_iter), '_', num2str(cost_sigma)];
    plotting_one_experiment(sdfmap_file, i_exp, img_name,'off');
    
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

% --------------------------------------- run best arguments --------------------------------------- 
args = [num2str(i_exp), ' ', num2str(best_args(1)), ' ', num2str(best_args(2)), ' ', num2str(best_args(3)), ' ', num2str(best_args(4)), ...
    ' ', num2str(best_args(5)), ' ', num2str(best_args(6)), ' ', num2str(best_args(7)), ' ', num2str(best_args(8)), ' ', ...
    num2str(best_args(9)), ' ', num2str(best_args(10))];
    
command = ['/home/hongzhe/git/VIMP/vimp/build/pgcs_PlanarPRModel', ' ', args];
num_iter = system(command)


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
