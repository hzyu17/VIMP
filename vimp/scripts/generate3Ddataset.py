import numpy as np
import os, sys

this_file = os.path.abspath(__file__)
vimp_dir  = os.path.dirname(os.path.dirname(this_file))
third_party_dir = vimp_dir + "/3rdparty"

if third_party_dir not in sys.path:    
    sys.path.insert(0, third_party_dir)


from sensor3D_tools import OccpuancyGrid  # adjust import to where you defined it


def generate_3d_dataset(dataset_str: str) -> OccpuancyGrid:
    """
    Create an OccpuancyGrid populated according to one of several named presets.
    
    Args:
        dataset_str: one of
            'SmallDemo', '3dPRMap2', 'WAMDeskDataset',
            'FrankaDeskDataset', 'FrankaBoxDataset', 'FrankaBoxDatasetOffset',
            'PR2DeskDataset', 'PR2DeskDataset_closer',
            'PR2BookShelfDataset', 'PR2IndustrialDataset'
            
    Returns:
        An OccpuancyGrid with its map, origin, cell_size, and obstacles set.
    """
    # --- select parameters & obstacle lists based on dataset_str ---
    if dataset_str == 'SmallDemo':
        rows, cols, z = 500, 500, 500
        origin = np.array([-10.0, -10.0, -10.0])
        cell_size = 0.1
        obs = [
            ([250, 250, 250], [140, 100, 200]),
        ]
        
    elif dataset_str == '3dPRMap2':
        rows, cols, z = 500, 500, 500
        origin = np.array([-10.0, -10.0, -10.0])
        cell_size = 0.1
        obs = [
            ([200, 100, 100], [200, 100, 100]),
            ([250,  50, 350], [140,  80, 250]),
            ([ 75, 250, 300], [100, 120, 200]),
            ([300, 400, 150], [250, 180, 150]),
            ([250, 250, 450], [150, 150,  50]),
            ([250, 250, 250], [ 80,  80,  80]),
        ]
        
    elif dataset_str == 'WAMDeskDataset':
        rows, cols, z = 300, 300, 300
        origin = np.array([-1.5, -1.5, -1.5])
        cell_size = 0.01
        obs = [
            ([170, 220, 130], [140,  60,   5]),
            ([105, 195,  90], [ 10,  10,  80]),
            ([235, 195,  90], [ 10,  10,  80]),
            ([105, 245,  90], [ 10,  10,  80]),
            ([235, 245,  90], [ 10,  10,  80]),
            ([250, 190, 145], [ 60,   5, 190]),
            ([250,  90, 145], [ 60,   5, 190]),
            ([200, 190, 145], [ 40,   5, 190]),
            ([250, 140, 240], [ 60, 100,   5]),
            ([250, 140, 190], [ 60, 100,   5]),
            ([250, 140, 140], [ 60, 100,   5]),
            ([250, 140,  90], [ 60, 100,   5]),
        ]
        
    elif dataset_str == 'FrankaDeskDataset':
        rows, cols, z = 300, 300, 300
        origin = np.array([-1.75, -1.5, -1.0])
        cell_size = 0.01
        obs = [
            ([170, 220, 130], [140,  60,   5]),
            ([105, 195,  90], [ 10,  10,  80]),
            ([235, 195,  90], [ 10,  10,  80]),
            ([105, 245,  90], [ 10,  10,  80]),
            ([235, 245,  90], [ 10,  10,  80]),
            ([250, 190, 145], [ 60,   5, 190]),
            ([250,  90, 145], [ 60,   5, 190]),
            ([210, 190, 145], [ 30,   5, 190]),
            ([250, 140, 140], [ 60, 100,   5]),
            ([250, 140,  90], [ 60, 100,   5]),
        ]
        
    elif dataset_str == 'FrankaBoxDataset':
        rows, cols, z = 300, 300, 300
        origin = np.array([-1.5, -1.5,  0.0])
        cell_size = 0.01
        obs = [
            ([200, 140,  20], [ 20,  20,  40]),
            ([200, 140,  42], [ 60,  60,   4]),
        ]
        
    elif dataset_str == 'FrankaBoxDatasetOffset':
        rows, cols, z = 300, 300, 300
        origin = np.array([-1.5, -1.5, 0.0])
        cell_size = 0.01
        obs = [
            ([200, 150, 20], [20, 20, 40]),
            ([200, 150, 42], [60, 60,  4]),
        ]
        
    elif dataset_str == 'PR2DeskDataset':
        rows, cols, z = 300, 300, 300
        origin = np.array([-1.4, -1.3, -1.5])
        cell_size = 0.01
        obs = [
            ([170,  60, 130], [140,  60,   5]),
            ([105,  35,  90], [ 10,  10,  80]),
            ([235,  35,  90], [ 10,  10,  80]),
            ([105,  85,  90], [ 10,  10,  80]),
            ([235,  85,  90], [ 10,  10,  80]),
            ([250, 190, 145], [ 60,   5, 190]),
            ([250,  90, 145], [ 60,   5, 190]),
            ([200,  90, 145], [ 40,   5, 190]),
            ([250, 140, 240], [ 60, 100,   5]),
            ([250, 140, 190], [ 60, 100,   5]),
            ([250, 140, 140], [ 60, 100,   5]),
            ([250, 140,  90], [ 60, 100,   5]),
        ]
        
    elif dataset_str == 'PR2DeskDataset_closer':
        rows, cols, z = 300, 300, 300
        origin = np.array([-1.4, -1.3, -0.95])
        cell_size = 0.01
        obs = [
            ([170,  50,  60], [140,  60,   5]),
            ([105,  25,  30], [ 10,  10,  60]),
            ([235,  25,  30], [ 10,  10,  60]),
            ([105,  75,  30], [ 10,  10,  60]),
            ([235,  75,  30], [ 10,  10,  60]),
            ([245, 190,  90], [ 60,   5, 180]),
            ([245,  90,  90], [ 60,   5, 180]),
            ([245, 140, 185], [ 60, 100,   5]),
            ([245, 140, 135], [ 60, 100,   5]),
            ([245, 140,  85], [ 60, 100,   5]),
            ([245, 140,  35], [ 60, 100,   5]),
            ([205,  90,  90], [ 25,   5, 180]),
        ]
        
    elif dataset_str == 'PR2BookShelfDataset':
        rows, cols, z = 300, 300, 300
        origin = np.array([-1.55, -1.3, -0.95])
        cell_size = 0.01
        obs = [
            ([245, 195,  70], [ 60,   5, 135]),
            ([245,  75,  70], [ 60,   5, 135]),
            ([245, 135, 135], [ 60, 120,   5]),
            ([245, 135,  95], [ 60, 120,   5]),
            ([245, 135,  55], [ 60, 120,   5]),
            ([245, 135,  15], [ 60, 120,   5]),
            ([260, 135,  70], [ 30, 125, 135]),
        ]
        
    elif dataset_str == 'PR2IndustrialDataset':
        rows, cols, z = 300, 300, 300
        origin = np.array([-1.7, -1.3, -0.95])
        cell_size = 0.01
        obs = [
            ([170,  45,  65], [140,  60,   5]),
            ([170,  25,  75], [ 85,   5,  20]),
            ([210,  45,  75], [  5,  40,  22]),
            ([170,  45,  75], [  5,  40,  20]),
            ([130,  45,  75], [  5,  40,  22]),
            ([265, 165,  85], [ 40,   5,  35]),
            ([265, 105,  85], [ 40,   5,  35]),
            ([265, 135, 100], [ 40,  60,   5]),
            ([255, 135,  65], [ 60, 120,   5]),
        ]
        
    else:
        raise ValueError(f"No such dataset: {dataset_str!r}")
    
    # --- build and populate the grid ---
    grid = OccpuancyGrid(rows, cols, z, cell_size, origin=origin)
    for center, size in obs:
        grid.add_obstacle(np.array(center), np.array(size))
    return grid


if __name__ == '__main__':
    grid = generate_3d_dataset('WAMDeskDataset')
    
    grid.save_to_json("WAMDeskDataset.json")