close all
clear

import gtsam.*
import gpmp2.*

% 2 configurations
start_confs = [-0.8,   -1.70,   1.64,  1.29,   1.1, -0.106,    2.2;
               -0.9,  -1.70,   1.34,  1.19,   0.8,  -0.126,    2.5];
end_confs = [-0.0,    0.94,     0,     1.6,     0,   -0.919,   1.55;
              -0.7,   1.35,    1.2,    1.0,   -0.7,  -0.1,   1.2];

for i_exp = 1:2 

    %% dataset
    dataset = generate3Ddataset('WAMDeskDataset');
    origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
    origin_point3 = Point3(origin');
    cell_size = dataset.cell_size;

    % sdf
    disp('calculating signed distance field ...');
    field = signedDistanceField3D(dataset.map, dataset.cell_size);
    disp('calculating signed distance field done');

    % arm: WAM arm
    arm = generateArm('WAMArm');

    start_conf = start_confs(i_exp,1:end)';
    end_conf = end_confs(i_exp,1:end)';
    start_vel = zeros(7,1);
    end_vel = zeros(7,1);

    % plot problem setting
    figure;
    tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
    nexttile
    hold on
    ttl = ['Problem Settings ', num2str(i_exp)];
    title(ttl)
    plotMap3D(dataset.corner_idx, origin, cell_size);
    plotRobotModel(arm, start_conf)
    plotRobotModel(arm, end_conf)
    % plot config
    set3DPlotRange(dataset)
    grid on, view(3)
    hold off

end