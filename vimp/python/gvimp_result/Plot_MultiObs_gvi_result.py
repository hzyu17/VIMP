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
import matplotlib

from sdf_robot.scripts.generate_sdf_2d import *
from sdf_robot.scripts.collision_costs_2d import *
from tools.draw_pquadsdf_trj import *

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
        
if __name__ == '__main__':
    sdf_2d, planarmap = generate_2dsdf("MultiObstacleMap", True)
    n_balls = 5
    L = 5.0
    H = 0.35
    i = 4
    
    # case_dirs = [os.path.join(result_dir, f'case{i+1}') for i in range(4)]
    case_dir = os.path.join(os.path.join(result_dir, 'Multi_Obs'), 'Go_around')
    # case_dir = os.path.join(os.path.join(result_dir, 'Multi_Obs'), 'Go_through')

    # fig, axes = plt.subplots(2, 2, frameon=False)

    mean_pd = pd.read_csv(os.path.join(case_dir,'zk_sdf.csv'), header=None)
    mean = mean_pd.values.T

    cov_pd = pd.read_csv(os.path.join(case_dir,'Sk_sdf.csv'), header=None)
    cov = cov_pd.values.T

    fig, ax = plt.subplots()

    fig, ax = planarmap.draw_map(fig, ax, plot=False)
    fig, ax = draw_pquad_trj_cov_2d(mean, cov, L, H, n_balls, fig, ax, 1, False, False)
    ax.set_xlim(-12.5, 27.5)
    ax.set_ylim(-2.5, 40)

    ax.axis('off') 
    ax.set_title('Go Around Plan', fontsize=18, fontweight='bold', fontdict={'family': 'serif'})
    # ax.set_title('Go Through Plan', fontsize=18, fontweight='bold', fontdict={'family': 'serif'})

    # ax.tick_params(axis='both', which='major', labelsize=16)

    # ax.legend(loc='best', prop={'family': 'serif', 'size': 18})
    
    plt.tight_layout() 

    # fig.savefig(case_dir+f'/Go_Around_updated.pdf', format='pdf', dpi=2000, bbox_inches='tight')


    plt.show()  
