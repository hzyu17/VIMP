function [] = plot_samples_WAM(dataset, origin, cell_size, arm, means, ...
                               covs, start_indx,end_indx, step_size, start_conf, end_conf)
%PLOT_SAMPLES_WAM Plot mean trajectory and samples from the joint
% trajectory distribution.
% Hongzhe Yu


addpath('/usr/local/gtsam_toolbox')
addpath ('/home/hongzhe/git/VIMP/matlab_helpers/experiments/WAM/utils')

import gtsam.*
import gpmp2.*

gcf;

for j = start_indx:step_size:end_indx
        nexttile
        hold on 
        view(-14.7458, 9.8376);
        plotMap3D(dataset.corner_idx, origin, cell_size);
        
        plotArm3D(arm.fk_model(), start_conf, 'r', 6, true);
        plotArm3D(arm.fk_model(), end_conf, 'g', 6, true);
        
        % gradual changing colors
        alpha_samples = 0.2;
        color = [0, 0, 1, 1];
        color_samples = [0, 0, 1, alpha_samples];
        % sample from covariance
        n_samples = 10;
        for i_sample = 1:n_samples
            % mu j
            mean_j = means(1:7, j);
            % cov j
            cov_j = covs(1:7, 1:7, j);

            % means
            plotArm3D(arm.fk_model(), mean_j, color, 8, true);

            % sampling 
            rng('default')  % For reproducibility
            samples = mvnrnd(mean_j, cov_j, n_samples);
            for k = 1: size(samples, 1)
                k_sample = samples(k, 1:end)';
                plotArm3D(arm.fk_model(), k_sample, color_samples, 4, false);
            end
        end
        
        xlim([-1, 1.5])
        ylim([-0.8, 1.5])
        hold off
        axis off

end
    
end

