function save_field_to_json(dataset, json_file)
% Save the constructed 3D field data set to json file. A python script can
% re-construct the 3D field by reading the json file.
% Saved informtion about the field:
% cols, rows, z, origin, cell_size, obstacles: corner_idx

meta.cols       = dataset.cols;
meta.rows       = dataset.rows;
meta.z      = dataset.z;
meta.origin     = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
meta.cell_size  = dataset.cell_size;

% -------------------------------------------------------------
% 2. obstacles  (corner_idx : N × 6)
%    [xmin xmax  ymin ymax  zmin zmax]  in voxel indices
% -------------------------------------------------------------
if isempty(dataset.corner_idx)
    meta.obstacles = {};
else
    Nobs = size(dataset.corner_idx,1);
    meta.obstacles = repmat(struct, Nobs, 1);

    for k = 1:Nobs
        c = dataset.corner_idx(k,:);
        meta.obstacles(k).corner_idx  = c;              % keep raw indices
    end
end

% -------------------------------------------------------------
% 3. encode + write to disk
% -------------------------------------------------------------
json_str = jsonencode(meta);   
fid = fopen(json_file,'w');
fwrite(fid, json_str, 'char');
fclose(fid);

fprintf('wrote %s  (%u obstacles)\n', json_file, numel(meta.obstacles));

end

