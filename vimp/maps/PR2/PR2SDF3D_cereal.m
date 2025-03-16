% Example to save a 3D sdf dataset
close all
clear   

addpath("../../../matlab_helpers/tools")
addpath("..")

%% dataset
dataset = generate3Ddataset_1('PR2DeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
cell_size = dataset.cell_size;

% init sdf
field = signedDistanceField3D(dataset.map, dataset.cell_size);

sdf = SignedDistanceField_mex('new', origin, cell_size, size(field, 1), size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    SignedDistanceField_mex('initFieldData', sdf, z-1, field(:, :, z));
end

% plot 3D SDF
x0 = 50;
y0 = 50;
width = 800;
height = 550;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
t.FontSize = 14;
grid on
hold on 
view(3)
plotMap3D(dataset.corner_idx, origin, cell_size);
xlim([0.2, 1.4])
ylim([-1, 0.8])
zlim([-0, 0.9])

%% save SDF
disp('saving sdf to .bin file...');
SignedDistanceField_mex('save', sdf, 'PR2DeskDataset_cereal.bin');

% %% create the mesh for visualization
% [X,Y,Z] = meshgrid(-10:1:20,-10:1:20,-10:1:20);
% X = reshape(X, [31*31*31, 1]);
% Y = reshape(Y, [31*31*31, 1]);
% Z = reshape(Z, [31*31*31, 1]);
% csvwrite("gridX.csv", X);
% csvwrite("gridY.csv", Y);
% csvwrite("gridZ.csv", Z);

% %% read mesh hinge 3D
% max(meshhinge3D)
