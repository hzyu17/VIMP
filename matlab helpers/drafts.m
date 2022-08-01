%% draft for debugging
precision  = csvread("../vimp/precision.csv")
cov = csvread("../vimp/cov.csv")


%%
diff = inv(precision) - cov;
norm(diff);

true_cov = inv(precision);
writematrix(true_cov, "cov_expected.csv");

%% matrices during the iterations

for i = 0:6
%     name = "../vimp/data/debug/Vddmu_"+num2str(i)+".csv";
%     Vddmu = csvread(name)
%     name = "../vimp/data/debug/j_Vddmu_"+num2str(i)+".csv"
%     j_Vddmu = csvread(name)
%     name = "../vimp/data/debug/dprecision_"+num2str(i)+".csv"
%     dprecision =  csvread(name)
    name = "../vimp/data/debug/dmu_"+num2str(i)+".csv"
    dmu =  csvread(name)
    name = "../vimp/data/debug/precision_"+num2str(i)+".csv"
    precision =  csvread(name)
    name = "../vimp/data/debug/cov_"+num2str(i)+".csv"
    cov=  csvread(name)
    aa = 1;
end

%% test gpmp2 functions
% Load libraries
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*
dataset = generate2Ddataset('MultiObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% plot sdf
figure(2)
plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
title('Signed Distance Field')
