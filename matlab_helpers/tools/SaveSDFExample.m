% Example to save sdf as gpmp2 datatype
% @author Mustafa Mukadam
% @date   May 4, 2017

close all
clear

addpath('gtsam_toolbox')
import gtsam.*
import gpmp2.*

% addpath('utils')
%% dataset
dataset = generate3Ddataset_1('FrankaBoxDataset');

save_field_to_json(dataset, 'FrankaBoxDataset.json');

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


% % plot sdf
figure;
ax = gca;        
eps_dist = 0.1;
marker_size = 10;
plotSDF3D(field, origin, cell_size, eps_dist, marker_size, ax);   

%% save SDF
disp('saving sdf to .bin file...');
sdf.saveSDF('FrankaBoxDataset.bin');
