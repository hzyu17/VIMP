function h = plotEvidenceMap2D_1(prob_grid, origin_x, origin_y, cell_size)
%PLOTEVIDENCEMAP2D Plot 2D evidence grid map, for 2D dataset visualization
%
%   Usage: PLOTEVIDENCEMAP2D(prob_grid, origin_x, origin_y, cell_size)
%   @prob_grid              evidence grid matrix
%   @origin_x, origin_y     origin (down-left) corner of the map
%   @cell_size              cell size

% map display setting
colormap([0.6 0.6 0.6; 0.7 0.7 0.7; 1 1 1]);
% axes('Color', 'none'); % Ensures that the axes background is transparent
% colormap([0.1 0.1 0.1; 
%           0.0 0.0 0.0; 
%           1 1 1]);

% get X-Y coordinates
grid_rows = size(prob_grid, 1);
grid_cols = size(prob_grid, 2);
grid_corner_x = origin_x + (grid_cols-1)*cell_size;
grid_corner_y = origin_y + (grid_rows-1)*cell_size;
grid_X = origin_x : cell_size : grid_corner_x;
grid_Y = origin_y : cell_size : grid_corner_y;

grid_Z = (1-prob_grid)*2+1;
alphaData = 0.5 * ones(grid_rows, grid_cols);  % Uniform transparency layer
alphaData(grid_Z(:, :) == 3) = 0;  % Set transparent where color is white

h = image(grid_X, grid_Y, grid_Z);
set(h, 'AlphaData', alphaData);

set(gca,'YDir','normal')

ax = gca; % current axes
% ax.TickDir = 'none';

axis equal
% axis([origin_x-cell_size/2*1.2, grid_corner_x+cell_size/2*1.2, ...
%           origin_y-cell_size/2*1.2, grid_corner_y+cell_size/2*1.2])
axis([-18, 18, -10, 25])

% set(gcf, 'Color', 'none');
end

