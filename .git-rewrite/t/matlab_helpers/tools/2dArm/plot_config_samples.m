function [] = plot_config_samples(sdfmap, arm, means, covs, start_indx,end_indx, step_size, start_conf, end_conf)
%PLOT_CONFIG_SAMPLES Plot sampled configurations and in work space.
%   Hongzhe Yu

addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

output = 1;
origin_x = -1;
origin_y = -1;
cell_size = 0.01;

% gcf;

for j = start_indx:step_size:end_indx
    nexttile
    hold on 
    
    plotEvidenceMap2D_arm(sdfmap, origin_x, origin_y, cell_size);
    
    plotPlanarArm(arm.fk_model(), start_conf, 'r', 10);
    plotPlanarArm(arm.fk_model(), end_conf, 'g', 10);
    
    % gradual changing colors
    alpha_samples = 0.02;
    color = [0, 0, 1, 1];
    color_samples = [0, 0, 1, alpha_samples];
    % sample from covariance
    n_samples = 10;
    for i_sample = 1:n_samples
        % mu j
        mean_j = means(1:2, j);
        % cov j
        cov_j = covs(1:2, 1:2, j);
        
        % plot means
        plotPlanarArm1(arm.fk_model(), mean_j, color, 10, true);
    
        % sampling 
        rng('default')  % For reproducibility
        samples = mvnrnd(mean_j, cov_j, n_samples);
        for k = 1: size(samples, 1)
            k_sample = samples(k, 1:end)';
            plotPlanarArm1(arm.fk_model(), k_sample, color_samples, 6, false);
        end
    end
    
    xlim([-1, 1.5])
    ylim([-0.8, 1.5])
    hold off
    axis off

end

end

