clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

%% choose the experiment and temperature to plot
% plot_temperture = "low";
plot_temperture = "high";

% plot_experiment = "map1_above";
% plot_experiment = "map1_below";
% plot_experiment = "map2_exp1";
plot_experiment = "map2_exp2";
% plot_experiment = "map2_exp3";
% plot_experiment = "map2_exp4"; 
% plot_experiment = "map_narrow_go_through";
% plot_experiment = "map_narrow_go_around";

prefix = "";
if strcmp(plot_experiment, "map1_above")
    sdfmap = csvread("map1/map_multiobs_map1.csv");
    prefix = ["map1/above_case/"];
elseif strcmp(plot_experiment, "map1_below")
    sdfmap = csvread("map1/map_multiobs_map1.csv");
    prefix = ["map1/below_case/"];
elseif strcmp(plot_experiment, "map2_exp1")
    sdfmap = csvread("map2/map_multiobs_map2.csv");
    prefix = ["map2/exp1/"];
elseif strcmp(plot_experiment, "map2_exp2")
    sdfmap = csvread("map2/map_multiobs_map2.csv");
    prefix = ["map2/exp2/"];
elseif strcmp(plot_experiment, "map2_exp3")
    sdfmap = csvread("map2/map_multiobs_map2.csv");
    prefix = ["map2/exp3/"];
elseif strcmp(plot_experiment, "map2_exp4")
    sdfmap = csvread("map2/map_multiobs_map2.csv");
    prefix = ["map2/exp4/"];
elseif strcmp(plot_experiment, "map_narrow_go_through")
    sdfmap = csvread("map3/map_multiobs_entropy_map3.csv");
    prefix = ["map3/shortcut/"];
elseif strcmp(plot_experiment, "map_narrow_go_around")
    sdfmap = csvread("map3/map_multiobs_entropy_map3.csv");
    prefix = ["map3/circumvent/"];
end

if strcmp(plot_temperture, "low") && (~contains(plot_experiment, "go_around"))
    means = csvread([prefix + "mean_base.csv"]);
    covs = csvread([prefix + "cov_base.csv"]);
    precisions = csvread([prefix + "precisoin_base.csv"]);
    costs = csvread([prefix + "cost_base.csv"]);
    
    factor_costs = csvread([prefix + "factor_costs_base.csv"]);
    perturb_stat= csvread([prefix + "perturbation_statistics_base.csv"]);
    final_cost = csvread([prefix + "final_cost_base.csv"]);
    
elseif strcmp(plot_temperture, "high")
    means = csvread([prefix + "mean.csv"]);
    covs = csvread([prefix + "cov.csv"]);
    precisions = csvread([prefix + "precisoin.csv"]);
    costs = csvread([prefix + "cost.csv"]);
    
    factor_costs = csvread([prefix + "factor_costs.csv"]);
    if ~(contains(plot_experiment, "narrow") || contains(plot_experiment, "map2"))
        perturb_stat= csvread([prefix + "perturbation_statistics.csv"]);
    end
    final_cost = csvread([prefix + "final_cost.csv"]);
end

addpath("error_ellipse");

output = plotPointRobotAnalysis(means, covs, precisions, costs, factor_costs, sdfmap);

