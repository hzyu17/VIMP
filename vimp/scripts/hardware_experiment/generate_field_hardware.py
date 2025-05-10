# Generate SDF for the hardware experiment
from vimp.thirdparty.sensor3D_tools import SignedDistanceField, SignedDistanceField3D, OccpuancyGrid
import yaml
import numpy as np



def generate_field_from_yaml(yaml_file):
    with open(yaml_file,"r") as f:
        cfg = yaml.safe_load(f)
    
    # Create 3D field    
    field = cfg.get("Field", [])
    field_origin = field["field_origin"]
    field_size   = field["field_size"]
    cell_size    = field["cell_size"]
    grid = OccpuancyGrid(field_size[0], field_size[1], field_size[2], cell_size, origin=field_origin)
    obs = []
    
    for center, size in obs:
        grid.add_obstacle(np.array(center), np.array(size))

    # 2) Access the body frame list to get the positions of the obstacles
    body_frames = cfg.get("Body_frames", [])
    
    # 3) Default box obstacle poses: assume to have parallel edges as the axis
    for bf in body_frames:
        position       = bf["position"]      
        orientation    = bf["orientation"]   
        
        
        
        box = bf["box"]
        box_center     = box["center"]        
        box_orientation= box["orientation"]   
        box_size       = box["size"]          
        cell_size      = box["cell_size"]
    
    
    
    field3d = SignedDistanceField3D.generate_field3D(grid.map.detach().numpy(), grid.cell_size)
    
    origin = np.array([
        grid.origin_x,
        grid.origin_y,
        grid.origin_z
    ], dtype=np.float64)

    sdf = SignedDistanceField(origin, grid.cell_size,
                                field3d.shape[0], field3d.shape[1], field3d.shape[2])
    for z in range(field3d.shape[2]):
        sdf.initFieldData(z, field3d[:,:,z])