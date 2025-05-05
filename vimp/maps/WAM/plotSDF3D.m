function plotSDF3D(field, origin, cell_size, epsilon_dist, marker_size, ax)
%PLOTSIGNEDDISTANCEFIELD3D  Visualise a 3-D signed-distance field.
%
%   plotSignedDistanceField3D(field, origin, cell_size)
%   plotSignedDistanceField3D(field, origin, cell_size, epsilon_dist)
%   plotSignedDistanceField3D(field, origin, cell_size, epsilon_dist, ...
%                             marker_size, ax)
%
%   INPUTS
%   ------
%   field         (Ny-by-Nx-by-Nz)  Numeric SDF array (inside < 0, outside > 0)
%   origin        [x0 y0 z0]        World-frame co-ordinate of voxel (1,1,1)
%   cell_size     scalar            Voxel edge length (same for x,y,z)
%   epsilon_dist  scalar  (opt.)    Plot every voxel whose SDF < epsilon_dist
%                                   (default 0 → exact interior)
%   marker_size   scalar  (opt.)    Marker size passed to SCATTER3 (default 10)
%   ax            axes    (opt.)    Axes to draw in (default = gca)
%
%   The function scatters the centres of all voxels whose signed-distance
%   value is below `epsilon_dist`, and sets a cuboid bounding box that
%   matches the volume limits.

    % ---------------- default arguments ----------------
    if nargin < 4 || isempty(epsilon_dist), epsilon_dist = 0;     end
    if nargin < 5 || isempty(marker_size), marker_size = 10;      end
    if nargin < 6 || isempty(ax),            ax = gca;            end

%     % ---------------- grid extents ---------------
    [Nx, Ny, Nz]     = size(field);
    [xIdx, yIdx, zIdx] = ndgrid(0:Ny-1, 0:Nx-1, 0:Nz-1);   % NOTE: rows=Y, cols=X
    Xw = origin(1) + yIdx(:)*cell_size;    % X = column index
    Yw = origin(2) + xIdx(:)*cell_size;    % Y = row    index
    Zw = origin(3) + zIdx(:)*cell_size;    % Z = slice  index
    D  = field(:);
    
    % ---------- Only showing the distance less than the epsilon ----------
    mask = abs(D) <= epsilon_dist;              % comment this line to show all points
    Xw = Xw(mask);  Yw = Yw(mask);  Zw = Zw(mask);  D = D(mask);

    % ---------------- scatter plot ----------------
    scatter3(ax, Xw, Yw, Zw, marker_size, D, 'filled');
    
    colormap(parula);
    
    colorbar;
    
    hold(ax, 'on');
    view(ax, 3);           % 3-D view
    axis(ax, 'equal');

%     axis(ax, [ origin(1)-cell_size/2, corner_x+cell_size/2, ...
%     origin(2)-cell_size/2, corner_y+cell_size/2, ...
%     origin(3)-cell_size/2, corner_z+cell_size/2 ]);

    xlabel(ax, 'X / m');
    ylabel(ax, 'Y / m');
    zlabel(ax, 'Z / m');
    title(ax, 'Signed Distance Field');
end

%% Example usage
% figure;
% ax = gca;        
% eps_dist = 0.1;
% marker_size = 2;
% plotSDF3D(field, origin, cell_size, eps_dist, marker_size, ax);   
