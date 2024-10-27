import os
import sys

file_path = os.path.abspath(__file__)
costfunction_dir = os.path.dirname(file_path)

py_dir = os.path.abspath(os.path.join(os.path.join(file_path, '..'),'..'))
collision_cost_dir = os.path.abspath(os.path.join(os.path.join(py_dir, 'sdf_robot'), 'scripts'))
VIMP_dir = os.path.abspath(os.path.join(os.path.join(py_dir, '..'), '..'))
result_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(VIMP_dir, 'matlab_helpers'), 'GVIMP-examples'), '2d_Quad'))

sys.path.append(collision_cost_dir)
sys.path.append(py_dir)
sys.path.append(result_dir)

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from sdf_robot.scripts.generate_sdf_2d import *
from sdf_robot.scripts.collision_costs_2d import *
from tools.draw_pquadsdf_trj import *

        
if __name__ == '__main__':
    # obstacle range: (x: [10.0, 20.0]; y: [10.0, 16.0])
    sdf_2d, planarmap = generate_2dsdf("SingleObstacleMap", True)
    n_balls = 5
    L = 5.0
    H = 0.35

    case_dirs = [os.path.join(result_dir, f'case{i+1}') for i in range(4)]
    fig, axes = plt.subplots(2, 2, frameon=False)

    for i, case_dir in enumerate(case_dirs):
        mean_pd = pd.read_csv(os.path.join(case_dir,'zk_sdf.csv'), header=None)
        mean = mean_pd.values.T

        cov_pd = pd.read_csv(os.path.join(case_dir,'Sk_sdf.csv'), header=None)
        cov = cov_pd.values.T

        ax = axes[i // 2, i % 2]
        fig, ax = planarmap.draw_map(fig, ax, plot=False)
        fig, ax = draw_pquad_trj_cov_2d(mean, cov, L, H, n_balls, fig, ax, 1, False, False)

        ax.legend(loc='best', prop={'family': 'serif', 'size': 12})
        # ax.set_title(f'Case {i+1}')
    
    plt.tight_layout() 
    plt.show()
