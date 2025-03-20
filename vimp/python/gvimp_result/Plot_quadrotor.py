import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import matplotlib

file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(file_path)
py_dir = os.path.abspath(os.path.join(current_dir, '..'))
map_dir = os.path.abspath(os.path.join(current_dir, '../sdf_robot/map/planar'))

result_dir = os.path.abspath(os.path.join(current_dir, '../../../matlab_helpers/GVIMP-examples/2d_Quad'))
# result_dir = os.path.abspath(os.path.join(current_dir, '../../../matlab_helpers/ProxKL-examples/2d_Quad'))

sys.path.append(py_dir)

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

from tools.draw_pquadsdf_trj import *
from sdf_robot.scripts.generate_sdf_2d import map2d

# json_file_path = os.path.join(map_dir, 'MultiObstacleLongRangeMap_params.json')
# json_file_path = os.path.join(map_dir, 'SingleObstacleMap_params.json')
json_file_path = os.path.join(map_dir, 'MultiObstacleMap_params.json')


with open(json_file_path, 'r') as f:
    params = json.load(f)

# Extract map parameters from JSON
map_name   = params.get("map_name", "DefaultMap")
origin     = np.array(params.get("origin", [0, 0]), dtype=np.float64)
cell_size  = params.get("cell_size", 0.1)
map_width  = params.get("map_width", 100)
map_height = params.get("map_height", 100)

# Create map object using map2d class
m = map2d(origin, cell_size, map_width, map_height, map_name)

# Add all obstacles defined in the JSON file
for obs in params.get("obstacles", []):
    m.add_box_xy(obs["xmin"], obs["ymin"], [obs["width"], obs["height"]])


# ----------------- 2. Read trajectory data CSV files -----------------
# case_dir = os.path.join(result_dir, 'case2_300_states')
# case_dir = os.path.join(result_dir, 'case2')
case_dir = os.path.join(result_dir, 'Multi_Obs/Go_around')

# Paths to the CSV files for trajectory mean and covariance data (no headers)
mean_csv_path = os.path.join(case_dir, 'zk_sdf.csv')
cov_csv_path  = os.path.join(case_dir, 'Sk_sdf.csv')

try:
    mean_pd = pd.read_csv(mean_csv_path, header=None)
    cov_pd  = pd.read_csv(cov_csv_path, header=None)
except Exception as e:
    print(f"Failed to read CSV files: {e}")
    sys.exit(1)

# Transpose data to match the original code
mean = mean_pd.values.T
cov  = cov_pd.values.T

# Set trajectory drawing parameters
n_balls = 5
L = 5.0
H = 0.35

# ----------------- 3. Draw map and trajectory -----------------
fig, ax = plt.subplots()

# First, draw the map (show axes, labels, etc.)
# fig, ax = m.draw_map(fig, ax, plot=False, labels=True)
fig, ax = m.draw_map(fig, ax, plot=False)

# Then overlay the trajectory on the map
fig, ax = draw_pquad_trj_cov_2d(mean, cov, L, H, n_balls, fig, ax, 1, False, False)
# ax.set_xlim(-7.5, 32.5)
# ax.set_ylim(-5, 40)

# ax.tick_params(axis='both', which='major', labelsize=16)
# ax.legend(loc='best', prop={'family': 'serif', 'size': 16})

# ax.axis('off')
# ax.set_title("Iteration 10", fontsize=18)

plt.tight_layout()
plt.show()

# fig.savefig(case_dir+f'/Nonlinear_Trajectory_10.pdf', format='pdf', dpi=2000, bbox_inches='tight')
