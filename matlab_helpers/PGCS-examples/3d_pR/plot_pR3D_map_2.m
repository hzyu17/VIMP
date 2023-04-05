clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*
addpath("../")

%% generate SDF
% dataset
dataset = generate3Ddataset_1('SmallDemo');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% init sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

%% plot 3D SDF
x0 = 50;
y0 = 50;
width = 800;
height = 550;
figure
set(gcf,'position',[x0,y0,width,height])

t.FontSize = 14;
view(3)

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')

for i = 1:1 % 4 experiments
    nexttile
    grid on
    hold on 
    prefix = ["map2/case"+num2str(i)+"/"];
    % % --- high temperature ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
   
    addpath("../error_ellipse");
    addpath("../../../matlab_helpers/");
    
    plotMap3D(dataset.corner_idx, origin, cell_size);
    
    plot_3d_result(means, covs);

    xlim([-5, 5])
    ylim([-5, 5])
    zlim([-5, 5])

end
