clear all
close all

addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

% sdf map
sdfmap = csvread("RAL-examples/2d_pR/map1/map_multiobs.csv");

colors = [255, 0, 0];
cell_size = 0.1;
origin_x = -20;
origin_y = -10;
origin_point2 = Point2(origin_x, origin_y);
field = signedDistanceField2D(sdfmap, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

grid_rows = size(sdfmap, 1);
grid_cols = size(sdfmap, 2);
grid_corner_x = origin_x + (grid_cols-1)*cell_size;
grid_corner_y = origin_y + (grid_rows-1)*cell_size;
grid_X = origin_x : cell_size : grid_corner_x;
grid_Y = origin_y : cell_size : grid_corner_y;

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
t.FontSize = 16;
hold on 
plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);

% plot results
zk_star = csvread("/home/hongzhe/git/VIMP/vimp/tests/zk_sdf.csv")
Sk_star = csvread("/home/hongzhe/git/VIMP/vimp/tests/Sk_sdf.csv")

nt = size(zk_star, 2);
zk_pos = zk_star(1:2, :);
Sk_star = reshape(Sk_star, 4,4,nt);
for i=1:nt
    scatter(zk_pos(1, i), zk_pos(2, i), 20, 'k', 'fill');
    error_ellipse(Sk_star(1:2,1:2,i), zk_pos(1:2, i));
end
