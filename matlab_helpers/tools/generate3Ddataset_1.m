function dataset = generate3Ddataset_1(dataset_str)
%GENERATE3DDATASET enerate 3D dataset evidence grid
%
%   Usage: dataset = GENERATE3DDATASET(dataset_str)
%   @dataset_str       dataset string, existing datasets:
%                      'WAMDeskDataset'
%
%   Dataset Format:
%   dataset.map        ground truth evidence grid
%   dataset.rows       number of rows (x)
%   dataset.cols       number of cols (y)
%   dataset.z          number of depth (z)
%   dataset.origin_x   origin of map x
%   dataset.origin_y   origin of map y
%   dataset.origin_z   origin of map z
%   dataset.cell_size  cell size
%   dataset.corner_idx corner index to visualize edges


% dataset 1: small dataset for demo
if strcmp(dataset_str, 'SmallDemo')
    % params
    dataset.cols = 500;
    dataset.rows = 500;
    dataset.z = 500;
    dataset.origin_x = -10;
    dataset.origin_y = -10;
    dataset.origin_z = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    [dataset.map, dataset.corner_idx] = add_obstacle([250 250 250], [140, 100, 200], dataset.map, dataset.corner_idx);
     
elseif strcmp(dataset_str, '3dPRMap2')
    % params
    dataset.cols = 500;
    dataset.rows = 500;
    dataset.z = 500;
    dataset.origin_x = -10;
    dataset.origin_y = -10;
    dataset.origin_z = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    [dataset.map, dataset.corner_idx] = add_obstacle([200 100 100], [200, 100, 100], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 50 350], [140, 80, 250], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([75 250 300], [100, 120, 200], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([300 400 150], [250, 180, 150], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 250 450], [150, 150, 50], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 250 250], [80, 80, 80], dataset.map, dataset.corner_idx);

% dataset 2: desk dataset for WAM WAMDeskDataset
elseif strcmp(dataset_str, 'WAMDeskDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    dataset.origin_x = -1.5;
    dataset.origin_y = -1.5;
    dataset.origin_z = -1.5;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    [dataset.map, dataset.corner_idx] = add_obstacle([170 220 130], [140, 60, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([105 195 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([235 195 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([105 245 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([235 245 90], [10, 10, 80], dataset.map, dataset.corner_idx);

    
    [dataset.map, dataset.corner_idx] = add_obstacle([250 190 145], [60, 5, 190], dataset.map, dataset.corner_idx);   
    [dataset.map, dataset.corner_idx] = add_obstacle([250 90 145], [60, 5, 190], dataset.map, dataset.corner_idx);   
   
    [dataset.map, dataset.corner_idx] = add_obstacle([200 190 145], [40, 5, 190], dataset.map, dataset.corner_idx);   
%     [dataset.map, dataset.corner_idx] = add_obstacle([130 40 95], [60, 5, 190], dataset.map, dataset.corner_idx);   
 
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 240], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 190], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 140], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 90], [60, 100, 5], dataset.map, dataset.corner_idx);

elseif strcmp(dataset_str, 'FrankaDeskDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    dataset.origin_x = -1.75;
    dataset.origin_y = -1.5;
    dataset.origin_z = -1;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    [dataset.map, dataset.corner_idx] = add_obstacle([170 220 130], [140, 60, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([105 195 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([235 195 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([105 245 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([235 245 90], [10, 10, 80], dataset.map, dataset.corner_idx);

    
    [dataset.map, dataset.corner_idx] = add_obstacle([250 190 145], [60, 5, 190], dataset.map, dataset.corner_idx);   
    [dataset.map, dataset.corner_idx] = add_obstacle([250 90 145], [60, 5, 190], dataset.map, dataset.corner_idx);   
   
    [dataset.map, dataset.corner_idx] = add_obstacle([210 190 145], [30, 5, 190], dataset.map, dataset.corner_idx);   
%     [dataset.map, dataset.corner_idx] = add_obstacle([130 40 95], [60, 5, 190], dataset.map, dataset.corner_idx);   
 
    % [dataset.map, dataset.corner_idx] = add_obstacle([250 140 240], [60, 100, 5], dataset.map, dataset.corner_idx);
    % [dataset.map, dataset.corner_idx] = add_obstacle([250 140 190], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 140], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 90], [60, 100, 5], dataset.map, dataset.corner_idx);

elseif strcmp(dataset_str, 'FrankaBoxDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    dataset.origin_x = -1.5;
    dataset.origin_y = -1.5;
    dataset.origin_z = 0.0;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    [dataset.map, dataset.corner_idx] = add_obstacle([200 140 20], [20, 20, 40], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([200 140 42], [60, 60, 4], dataset.map, dataset.corner_idx);


elseif strcmp(dataset_str, 'FrankaBoxDatasetOffset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    dataset.origin_x = -1.5;
    dataset.origin_y = -1.5;
    dataset.origin_z = 0;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    [dataset.map, dataset.corner_idx] = add_obstacle([200 150 20], [20, 20, 40], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([200 150 42], [60, 60, 4], dataset.map, dataset.corner_idx);


elseif strcmp(dataset_str, 'PR2DeskDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    % The position of the PR2 Right Arm in ROS is at (-0.1, -0.2, 1)
    dataset.origin_x = -1.4;
    dataset.origin_y = -1.3;
    dataset.origin_z = -1.5;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    [dataset.map, dataset.corner_idx] = add_obstacle([170 60 130], [140, 60, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([105 35 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([235 35 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([105 85 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([235 85 90], [10, 10, 80], dataset.map, dataset.corner_idx);
    
    [dataset.map, dataset.corner_idx] = add_obstacle([250 190 145], [60, 5, 190], dataset.map, dataset.corner_idx);   
    [dataset.map, dataset.corner_idx] = add_obstacle([250 90 145], [60, 5, 190], dataset.map, dataset.corner_idx);   
   
    [dataset.map, dataset.corner_idx] = add_obstacle([200 90 145], [40, 5, 190], dataset.map, dataset.corner_idx);  
 
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 240], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 190], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 140], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 90], [60, 100, 5], dataset.map, dataset.corner_idx);

elseif strcmp(dataset_str, 'PR2DeskDataset_closer')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    % The position of the PR2 Right Arm in ROS is at (-0.1, -0.2, 1)
    dataset.origin_x = -1.4;
    dataset.origin_y = -1.3;
    dataset.origin_z = -0.95;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    % table
    [dataset.map, dataset.corner_idx] = add_obstacle([170 50 60], [140, 60, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([105 25 30], [10, 10, 60], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([235 25 30], [10, 10, 60], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([105 75 30], [10, 10, 60], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([235 75 30], [10, 10, 60], dataset.map, dataset.corner_idx);
    
    % shelf
    [dataset.map, dataset.corner_idx] = add_obstacle([245 190 90], [60, 5, 180], dataset.map, dataset.corner_idx);   
    [dataset.map, dataset.corner_idx] = add_obstacle([245 90 90], [60, 5, 180], dataset.map, dataset.corner_idx);   
 
    [dataset.map, dataset.corner_idx] = add_obstacle([245 140 185], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([245 140 135], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([245 140 85], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([245 140 35], [60, 100, 5], dataset.map, dataset.corner_idx);

    [dataset.map, dataset.corner_idx] = add_obstacle([205 90 90], [25, 5, 180], dataset.map, dataset.corner_idx); 
    
elseif strcmp(dataset_str, 'PR2BookShelfDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    % The position of the PR2 Right Arm in ROS is at (0, -0.2, 0.95)
    dataset.origin_x = -1.55;
    dataset.origin_y = -1.3;
    dataset.origin_z = -0.95;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];
    
    % shelf
    [dataset.map, dataset.corner_idx] = add_obstacle([245 195 70], [60, 5, 135], dataset.map, dataset.corner_idx);   
    [dataset.map, dataset.corner_idx] = add_obstacle([245 75 70], [60, 5, 135], dataset.map, dataset.corner_idx);   
 
    [dataset.map, dataset.corner_idx] = add_obstacle([245 135 135], [60, 120, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([245 135 95], [60, 120, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([245 135 55], [60, 120, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([245 135 15], [60, 120, 5], dataset.map, dataset.corner_idx);

    [dataset.map, dataset.corner_idx] = add_obstacle([260 135 70], [30, 125, 135], dataset.map, dataset.corner_idx);

elseif strcmp(dataset_str, 'PR2IndustrialDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    % The position of the PR2 Right Arm in ROS is at (0, -0.2, 0.95)
    dataset.origin_x = -1.7;
    dataset.origin_y = -1.3;
    dataset.origin_z = -0.95;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    % obstacles
    dataset.corner_idx = [];

    % table
    [dataset.map, dataset.corner_idx] = add_obstacle([170 45 65], [140, 60, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([170 25 75], [85, 5, 20], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([210 45 75], [5, 40, 22], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([170 45 75], [5, 40, 20], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([130 45 75], [5, 40, 22], dataset.map, dataset.corner_idx);
    
    % shelf
    [dataset.map, dataset.corner_idx] = add_obstacle([265 165 85], [40, 5, 35], dataset.map, dataset.corner_idx);   
    [dataset.map, dataset.corner_idx] = add_obstacle([265 105 85], [40, 5, 35], dataset.map, dataset.corner_idx);   
 
    [dataset.map, dataset.corner_idx] = add_obstacle([265 135 100], [40, 60, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([255 135 65], [60, 120, 5], dataset.map, dataset.corner_idx);

% no such dataset
else
    error('No such dataset exist');
end




end

function [map, corner] = add_obstacle(position, size, map, corner)

half_size_row = floor((size(1)-1)/2);
half_size_col = floor((size(2)-1)/2);
half_size_z = floor((size(3)-1)/2);

% occupency grid
map(position(1)-half_size_row : position(1)+half_size_row, ...
    position(2)-half_size_col : position(2)+half_size_col, ...
    position(3)-half_size_z   : position(3)+half_size_z) ...
    = ones(2*half_size_row+1, 2*half_size_col+1, 2*half_size_z+1); 

% corner
corner = [corner; ...
   [position(1)-half_size_row , position(1)+half_size_row, ...
    position(2)-half_size_col , position(2)+half_size_col,...
    position(3)-half_size_z   , position(3)+half_size_z]];

end
