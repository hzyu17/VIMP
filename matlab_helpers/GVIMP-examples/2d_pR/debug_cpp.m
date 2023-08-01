clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
addpath("error_ellipse");
import gtsam.*
import gpmp2.*

%% choose the experiment and temperature to plot
plot_temperture = "low";

prefix = "";
sdfmap = csvread("map1/map_multiobs_map1.csv");
prefix = ["map1/above_case/"];
prefix_debug = ["map1/case1/"];

if strcmp(plot_temperture, "low")
    means = csvread([prefix + "mean_base.csv"]);
    covs = csvread([prefix + "cov_base.csv"]);
    precisions = csvread([prefix + "precisoin_base.csv"]);
    costs = csvread([prefix + "cost.csv"]);
    factor_costs = csvread([prefix + "factor_costs.csv"]);
    
    means_cpp = csvread([prefix_debug + "mean.csv"]);
    covs_cpp = csvread([prefix_debug + "cov.csv"]);
    precisions_cpp = csvread([prefix_debug + "precisoin.csv"]);
    costs_cpp = csvread([prefix_debug + "cost.csv"]);
    factor_costs_cpp = csvread([prefix_debug + "factor_costs.csv"]);
    
    joint_covs_cpp = csvread([prefix_debug + "joint_cov.csv"]);
    joint_precisions_cpp = csvread([prefix_debug + "joint_precisoin.csv"]);
    
end

for i = 1:10
    precisions_i = precisions((i-1)*40+1:i*40, 1:40);
    cov_i = covs((i-1)*40+1:i*40, 1:40);

    joint_precisions_cpp_i = joint_precisions_cpp(1:end, i);
    joint_precisions_cpp_i = reshape(joint_precisions_cpp_i, [40,40]);
    
    joint_cov_cpp_i = joint_covs_cpp(1:end, i);
    joint_cov_cpp_i = reshape(joint_cov_cpp_i, [40,40]);
    
    mean_i = means(i, 1:end)';
    mean_cpp_i = means_cpp(1:end, i);
    
    factor_cost_i = factor_costs(i, 1:end)';
    factor_cost_cpp_i = factor_costs_cpp(1:end, i);
end

debug = 1;
