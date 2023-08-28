% Example to save a 3D sdf dataset

close all
clear

addpath("/usr/local/gtsam_toolbox")
import gtsam.*
import gpmp2.*

addpath("../../tools")                      
%% dataset
dataset = generate3Ddataset_1('3dPRMap2');
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
xlim([-20, 40])
ylim([-20, 40])
zlim([-10, 40])

%% save SDF
disp('saving sdf to .bin file...');
sdf.saveSDF('pRSDF3D.bin');

%% create the mesh for visualization
[X,Y,Z] = meshgrid(-10:1:20,-10:1:20,-10:1:20);
X = reshape(X, [31*31*31, 1]);
Y = reshape(Y, [31*31*31, 1]);
Z = reshape(Z, [31*31*31, 1]);
csvwrite("gridX.csv", X);
csvwrite("gridY.csv", Y);
csvwrite("gridZ.csv", Z);

%% read mesh hinge 3D
% max(meshhinge3D)
