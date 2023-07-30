clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
addpath("error_ellipse");
import gtsam.*
import gpmp2.*

%% choose the experiment and temperature to plot
plot_temperture = "low";
% plot_temperture = "high";

plot_experiment = "map1_above";

prefix = "";
if strcmp(plot_experiment, "map1_above")
    sdfmap = csvread("map1/map_multiobs_map1.csv");
    prefix = ["map1/above_case/"];
    prefix_debug = ["map1/case1/"];
end

if strcmp(plot_temperture, "low") && (~contains(plot_experiment, "go_around"))
    means = csvread([prefix + "mean.csv"]);
    covs = csvread([prefix + "cov.csv"]);
    precisions = csvread([prefix + "precisoin.csv"]);
    costs = csvread([prefix + "cost.csv"]);
    factor_costs = csvread([prefix + "factor_costs.csv"]);
    
    means_cpp = csvread([prefix_debug + "mean.csv"]);
    covs_cpp = csvread([prefix_debug + "cov.csv"]);
    precisions_cpp = csvread([prefix_debug + "precisoin.csv"]);
    costs_cpp = csvread([prefix_debug + "cost.csv"]);
    factor_costs_cpp = csvread([prefix_debug + "factor_costs.csv"]);
    
end

precisions_1 = precisions(1:40, 1:40);
precisions_cpp_1 = precisions_cpp();

debug = 1;
