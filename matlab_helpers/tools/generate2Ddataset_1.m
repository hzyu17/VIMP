function dataset = generate2Ddataset_1(dataset_str)
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


% dataset 1: 1 obs dataset for 2D Arm obs avoid
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

elseif strcmp(dataset_str, 'OneObstacleDatasetNoisy')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.origin_x = -1;
    dataset.origin_y = -1;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    
    % sample the obstacle location
    randN_1 = randn()*5;
    randN_2 = randn()*5;
    
    % obstacles
    dataset.map = add_obstacle([190+randN_1, 160+randN_2], [60, 80], dataset.map);
    
% dataset 2: multiple obs dataset for 2D Arm obs avoid
elseif strcmp(dataset_str, 'MultiObstacleDataset')
    % params
    dataset.cols = 400; %x
    dataset.rows = 300; %y
    dataset.origin_x = -20;
    dataset.origin_y = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle(get_center(12,10,dataset), get_dim(5,7,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(-8,10,dataset), get_dim(9,7,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(0,-5,dataset), get_dim(10,5,dataset), dataset.map);

% dataset 3: multiple obs dataset for 2D Point obs avoid
elseif strcmp(dataset_str, 'MultiObstacleEntropy1')
    % params
    dataset.cols = 400; %x
    dataset.rows = 300; %y
    dataset.origin_x = -20;
    dataset.origin_y = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle(get_center(12,10,dataset), get_dim(5,7,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(-2.5,10,dataset), get_dim(9,7,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(0,-5,dataset), get_dim(10,5,dataset), dataset.map);

% dataset 3: multiple obs dataset for 2D Point obs avoid
elseif strcmp(dataset_str, 'MultiObstacleEntropy2')
    % params
    dataset.cols = 400; %x
    dataset.rows = 300; %y
    dataset.origin_x = -20;
    dataset.origin_y = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle(get_center(12,0,dataset), get_dim(5,10,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(-12,0,dataset), get_dim(7,10,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(-10,13,dataset), get_dim(8,5,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(1,0,dataset), get_dim(10,5,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(5,12,dataset), get_dim(7,8,dataset), dataset.map);
    
% Map 2 with perception noise simulation
elseif strcmp(dataset_str, 'MultiObstacleEntropy2Noisy')
    % params
    dataset.cols = 400; %x
    dataset.rows = 300; %y
    dataset.origin_x = -20;
    dataset.origin_y = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    
    % Perception noise
    randN_1 = randn()*5;
    randN_2 = randn()*5;
    
    % obstacles
    dataset.map = add_obstacle(get_center(12,0,dataset)+randN_1, get_dim(5,10,dataset)+randN_2, dataset.map);
    dataset.map = add_obstacle(get_center(-12,0,dataset)+randN_1, get_dim(7,10,dataset)+randN_2, dataset.map);
    dataset.map = add_obstacle(get_center(-10,13,dataset)+randN_1, get_dim(8,5,dataset)+randN_2, dataset.map);
    dataset.map = add_obstacle(get_center(1,0,dataset)+randN_1, get_dim(10,5,dataset)+randN_2, dataset.map);
    dataset.map = add_obstacle(get_center(5,12,dataset)+randN_1, get_dim(7,8,dataset)+randN_2, dataset.map);
    
% Map narrow gap for 2D point robot motion planning
elseif strcmp(dataset_str, 'MultiObstacleEntropy3')
    % params
    dataset.cols = 400; %x
    dataset.rows = 300; %y
    dataset.origin_x = -20;
    dataset.origin_y = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle(get_center(-4,5,dataset), get_dim(6,15,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(4,5,dataset), get_dim(6,15,dataset), dataset.map);

elseif strcmp(dataset_str, 'MultiObstacleEntropy3Noisy')
    % The obstacle's y positions are sampled from a Gaussian distribution.
    % params
    dataset.cols = 400; %x
    dataset.rows = 300; %y
    dataset.origin_x = -20;
    dataset.origin_y = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % sample the obstacle location
    randN = randn();
    
    center_1 = get_center(-4,5,dataset);
    center_2 = get_center(4,5,dataset);
    
    center_1(2) = center_1(2) + randN*10;
    center_2(2) = center_2(2) + randN*10;
    
    dim_1 = get_dim(6,15,dataset);
    dim_2 = get_dim(6,15,dataset);
    
    % obstacles
    dataset.map = add_obstacle(center_1, dim_1, dataset.map);
    dataset.map = add_obstacle(center_2, dim_2, dataset.map);
    
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
