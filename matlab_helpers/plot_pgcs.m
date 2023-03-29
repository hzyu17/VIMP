clear all
close all

addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

map_choice = 2;
map2_case = 4;  % 4 cases in map2

prefix="map1";
map2_case_str = "";

switch map_choice
    case 1
        prefix = 'map1'
    case 2
        switch map2_case
            case 1
                map2_case_str = "/case1";
            case 2
                map2_case_str = "/case2";
            case 3
                map2_case_str = "/case3";
            case 4
                map2_case_str = "/case4";
        end
        prefix = 'map2'
    case 3
        prefix = "map3"
end

sdfmap_file = strcat("RAL-examples/2d_pR/",prefix,"/map_multiobs_",prefix,".csv")

% sdf map
sdfmap = csvread(sdfmap_file);

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
zk_file = strcat("/home/hongzhe/git/VIMP/vimp/data/pgcs/double_integrator/",prefix,map2_case_str,"/zk_sdf.csv");
Sk_file = strcat("/home/hongzhe/git/VIMP/vimp/data/pgcs/double_integrator/",prefix,map2_case_str,"/Sk_sdf.csv");
zk_star = csvread(zk_file);
Sk_star = csvread(Sk_file);

nt = size(zk_star, 2);
zk_pos = zk_star(1:2, :);
Sk_star = reshape(Sk_star, 4,4,nt);
for i=1:nt
    scatter(zk_pos(1, i), zk_pos(2, i), 20, 'k', 'fill');
    error_ellipse(Sk_star(1:2,1:2,i), zk_pos(1:2, i));
end
