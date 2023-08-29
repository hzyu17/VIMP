function dataset = generate2Ddataset_arm(dataset_str)
%GENERATE2DDATASET Generate 2D dataset evidence grid
%
%   Usage: dataset = GENERATE2DDATASET(dataset_str)
%   @dataset_str       dataset string, existing datasets:
%                      'OneObstacleDataset', 'TwoObstaclesDataset'
%
%   Output Format:
%   dataset.map        ground truth evidence grid
%   dataset.rows       number of rows (y)
%   dataset.cols       number of cols (x)
%   dataset.origin_x   origin of map x
%   dataset.origin_y   origin of map y
%   dataset.cell_size  cell size

% dataset 5: 1 obs dataset for 2D Arm obs avoid
if strcmp(dataset_str, 'OneObstacleDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.origin_x = -1;
    dataset.origin_y = -1;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle([190, 160], [60, 80], dataset.map);
    
% dataset 7: multiple obs dataset for 2D Arm obs avoid
elseif strcmp(dataset_str, 'MultiObstacleDatasetArm')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.origin_x = -1;
    dataset.origin_y = -1;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle([190, 160], [60, 80], dataset.map);
    dataset.map = add_obstacle([190, 60], [60, 30], dataset.map);

% no such dataset
else
    error('No such dataset exist');
end

end

function [map, landmarks] = add_obstacle(position, size, map, landmarks, origin_x, origin_y, cell_size)

half_size_row = floor((size(1)-1)/2);
half_size_col = floor((size(2)-1)/2);

% occupency grid
map(position(1)-half_size_row : position(1)+half_size_row, ...
    position(2)-half_size_col : position(2)+half_size_col) ...
    = ones(2*half_size_row+1, 2*half_size_col+1); 

% landmarks
if nargin == 7
    for x = position(1)-half_size_row-1 : 4 : position(1)+half_size_row-1
        y = position(2)-half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        y = position(2)+half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
    
    for y = position(2)-half_size_col+3 : 4 : position(2)+half_size_col-5
        x = position(1)-half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        x = position(1)+half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
end

end

function center = get_center(x,y,dataset)

center = [y - dataset.origin_y, x - dataset.origin_x]./dataset.cell_size;

end

function dim = get_dim(w,h,dataset)

dim = [h, w]./dataset.cell_size;

end
