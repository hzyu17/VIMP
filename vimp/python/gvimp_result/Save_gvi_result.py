import os
import sys

file_path = os.path.abspath(__file__)
costfunction_dir = os.path.dirname(file_path)

py_dir = os.path.abspath(os.path.join(os.path.join(file_path, '..'),'..'))
collision_cost_dir = os.path.abspath(os.path.join(os.path.join(py_dir, 'sdf_robot'), 'scripts'))
VIMP_dir = os.path.abspath(os.path.join(os.path.join(py_dir, '..'), '..'))
result_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(VIMP_dir, 'matlab_helpers'), 'GVIMP-examples'), '2d_Quad'))
# result_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(VIMP_dir, 'matlab_helpers'), 'ProxKL-examples'), '2d_Quad'))

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
    # obstacle range: (x: [10.0, 20.0]; y: [10.0, 16.0])
    sdf_2d, planarmap = generate_2dsdf("SingleObstacleMap", True)
    n_balls = 5
    L = 5.0
    H = 0.35
    i = 4
    
    # case_dir = os.path.join(result_dir, 'case2_300_states')
    case_dir = os.path.join(result_dir, 'case2')

    # fig, axes = plt.subplots(2, 2, frameon=False)

    mean_pd = pd.read_csv(os.path.join(case_dir,'zk_sdf.csv'), header=None)
    mean = mean_pd.values.T

    cov_pd = pd.read_csv(os.path.join(case_dir,'Sk_sdf.csv'), header=None)
    cov = cov_pd.values.T

    fig, ax = plt.subplots()

    fig, ax = planarmap.draw_map(fig, ax, plot=False)
    fig, ax = draw_pquad_trj_cov_2d(mean, cov, L, H, n_balls, fig, ax, 1, False, False)
    ax.set_xlim(-7.5, 32.5)
    ax.set_ylim(-5, 40)

    # ax.tick_params(axis='both', which='major', labelsize=16)
    # ax.legend(loc='best', prop={'family': 'serif', 'size': 16})

    # ax.axis('off') 
    # ax.set_title(r'Iteration 5, $\hat{T} = 90$', fontsize=20)
    
    plt.tight_layout() 

    # fig.savefig(case_dir+f'/Trajectory_2_30_150_updated.pdf', format='pdf', dpi=2000, bbox_inches='tight')

    # fig.savefig(case_dir+f'/Trajectory4_updated.pdf', format='pdf', dpi=2000, bbox_inches='tight')


    # print(matplotlib.get_cachedir())
    # print(matplotlib.rcParams['font.family'])
    # print(matplotlib.rcParams['pdf.fonttype'])
    # print(matplotlib.rcParams['ps.fonttype'])

    plt.show()  



    

    
