function output = plotArmSamples3D(arm, vec_means, vec_covs, n_states, niters, nsteps, dataset, start_conf, end_conf)
% Plot the mean and sampled supported states for a arm model.
% Author: Hongzhe Yu

%% ==================  plot for paper experiment: using the gradual changing colors ================
import gtsam.*
import gpmp2.*

origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
cell_size = dataset.cell_size;

% step sizes
step_size = floor(niters / nsteps);

x0 = 500;
y0 = 500;
width = 600;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, floor(nsteps/2), 'TileSpacing', 'tight', 'Padding', 'tight')
for i_iter = 1: nsteps
    nexttile
    t = title(['Iteration ', num2str(i_iter*step_size)]);
    t.FontSize = 16;
    hold on 
    grid on, view(3)
    i_vec_means = vec_means{i_iter};
    i_vec_covs = vec_covs{i_iter};
    hold on 
    plotMap3D(dataset.corner_idx, origin, cell_size);
    color_sample = [0.0, 0.0, 0.7, 0.02];
    % forward kinematics
    wam_fk_model = arm.fk_model();
    
    n_samples = 50;
    for j = 1:n_states
        % gradual changing colors
        alpha = (j / n_states)^(1.15);
        color = [0, 0, 1, alpha];

        % means
        mean_j = i_vec_means{j}';

        % cov j
        cov_j = i_vec_covs{j};
        
        % sampling 
        rng('default')  % For reproducibility
        samples = mvnrnd(mean_j, cov_j, n_samples);
        plotArm3D(arm.fk_model(), mean_j, color, 4, true);
        for k = 1: size(samples, 1)
            k_sample = samples(k, 1:end)';
            plotArm3D(arm.fk_model(), k_sample, color_sample, 3, false);
        end
        
        % plot start and end configurations
        plotArm3D(arm.fk_model(), start_conf, 'r', 3, true);
        plotArm3D(arm.fk_model(), end_conf, 'g', 3, true);
    end
end

output = true;
end