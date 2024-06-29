import sys
import os

exp_script_dir = os.path.dirname(__file__)
py_dir = os.path.abspath(os.path.join(os.path.join(exp_script_dir, '..'),'..'))
src_dir = os.path.abspath(os.path.join(py_dir, '..'))
root_dir = os.path.abspath(os.path.join(src_dir, '..'))
collision_cost_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(root_dir, '3rdparty'), 
                                                               'sdf_robot'), 'scripts'))

sys.path.append(collision_cost_dir)
sys.path.append(root_dir)
sys.path.append(py_dir)

from dynamics.planar_quad import *
from tools.draw_pquadsdf_trj import *
import numpy as np
from covariance_steering.prox_cov_sdf import *


if __name__ == '__main__':
    nx = 6
    nu = 2
    
    nt = 3000
    
    # ============================= 4 experiment settings =============================
    exp_index = 4
    
    if (exp_index==1):
        # -------
        # exp 1
        # -------
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        xT = np.array([25.0, 30.0, np.pi/6, -1.0, 2.0, 0.01], dtype=np.float64)
        eps_obs = 0.8
        slope = 5.0
        sig_obs = 80
        radius = 1.0
        tf = 4.0
    elif exp_index == 2:
        # -------
        # exp 2
        # -------
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        xT = np.array([17.0, 35.0, -np.pi/8, 1.0, 2.0, 0.01], dtype=np.float64)
        eps_obs = 0.8
        slope = 5.0
        sig_obs = 50
        radius = 1.2
        tf = 4.5
    elif exp_index == 3:
        # -------
        # exp 3
        # -------
        x0 = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        xT = np.array([5.0, 35.0, -np.pi/6, 1.0, 1.0, -0.02], dtype=np.float64)
        eps_obs = 0.8
        slope = 5.0
        sig_obs = 80
        radius = 1.0
        tf = 4.0
    elif exp_index == 4:
        # -------
        # exp 4
        # -------
        x0 = np.array([25.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        xT = np.array([12.0, 35.0, np.pi/6, -2.0, 1.0, 0.0], dtype=np.float64)
        eps_obs = 0.8
        slope = 5.0
        sig_obs = 85
        radius = 1.0
        tf = 4.0
    
    # ============================= End of 4 experiment settings =============================
    
    dt = tf / (nt-1)
    
    epsilon = 1.0
    n_iter = 20
    n_iter_linesearch = 5
    stepsize = 3e-1
    
    # ------------------------------- 
    # Plot controlled ltv trajectory
    # -------------------------------

    fig, ax = plt.subplots(1,1,frameon=False)
    # ax, ax2 = axes.flatten()
    sdf_2d, planarmap = generate_2dsdf("SingleObstacleMap", savemap=False)

    L = 5.0
    H = 0.35
    n_balls = 5
    
    # ------------
    # read data
    # ------------
    import pickle
    # Open the file for reading
    with open(exp_script_dir+'/four_exp_data.pkl', 'rb') as file:
        # Load the data from the file
        data_loaded = pickle.load(file)
    
    zkSk = data_loaded[exp_index]
    
    zkstar = zkSk["zk"]
    Skstar = zkSk["Sk"]
    
    # ================ Animation ================
    animate = True
        
    if animate:
        
        font = {'family': 'serif',
        'color':  'black',
        'weight': 'normal',
        'size': 16,
        }
        
        fig, ax = plt.subplots(1,1, figsize=(10, 10))
        fig.tight_layout()
    
        fig, ax = planarmap.draw_map(fig, ax, plot=False)
        ax.grid(True)
        
        if (exp_index==1):
            # ----- exp 1 -----
            ax.set_xlim([-5,30])
            ax.set_ylim([-5,35])
        elif (exp_index==2):
            # ----- exp 2 -----
            ax.set_xlim([-10,25])
            ax.set_ylim([-2,42])
        elif (exp_index==3):
            # ----- exp 3 -----
            ax.set_xlim([-5,28])
            ax.set_ylim([-2,38])
        elif (exp_index==4):
            # ----- exp 4 -----
            ax.set_xlim([0,35])
            ax.set_ylim([-2,38])
        
        step = 10
        
        fig, ax = draw_quad_balls(x0, L, H, n_balls, fig, ax, draw_ball=False, patch_color='r')
        fig, ax = draw_quad_balls(xT, L, H, n_balls, fig, ax, draw_ball=False, patch_color='g')
               
        plt.draw()
        
        plt.pause(2)
        
        for i in range(0, nt, step):
            # -------- plot planar quadrotor -------
            x = zkstar[i]
            fig, ax = draw_quad_balls(x, L, H, n_balls, fig, ax, draw_ball=False, patch_color='b')
            
            plt.draw()
            plt.pause(0.01)  # Pause for a fraction of a second
        
    # -------------------------
    # Plot map and mean trj
    # -------------------------
    fig, ax = draw_pquad_map_trj_2d(x0, xT, zkstar, 
                                    L, H, n_balls, 
                                    fig, ax, planarmap, 
                                    step = 20, 
                                    draw_ball=False, save_fig=False, file_name="pquad_trj2d.pdf")

    # ----------------------------
    # Plot state space covaraince
    # ----------------------------  
    font = {'family': 'serif',
        'color':  'black',
        'weight': 'normal',
        'size': 16,
        }

    from tools.plot_tube import plot_cov_tube, plot_cov_tube_3D   
    fig, ax = plot_cov_tube(zkstar[:,0:2], Skstar[:, 0:2, 0:2], tf, 'red', 20, fig, ax)
    # ax.set_title(r'PGCS-MP for Linear Time-Varying system', fontdict=font)
    ax.set_title("")
    ax.set_xlabel(r'$p_x$', fontdict=font)
    ax.set_ylabel(r'$p_z$', fontdict=font)
    ax.grid(True)

    if (exp_index==1):
        # ----- exp 1 -----
        ax.set_xlim([-5,30])
        ax.set_ylim([-5,35])
    elif (exp_index==2):
        # ----- exp 2 -----
        ax.set_xlim([-10,25])
        ax.set_ylim([-2,42])
    elif (exp_index==3):
        # ----- exp 3 -----
        ax.set_xlim([-5,28])
        ax.set_ylim([-2,38])
    elif (exp_index==4):
        # ----- exp 4 -----
        ax.set_xlim([0,35])
        ax.set_ylim([-2,38])

    ax.legend()
    fig.tight_layout()

    fig.savefig(exp_script_dir+"/figures/pquad_sdf.pdf", transparent=True, dpi=1100, bbox_inches='tight', pad_inches=0)

    # =================== 
    # covariance vx-vz
    # =================== 
    fig2 = plt.figure(frameon=False)
    ax2 = fig2.add_subplot(111, projection='3d')

    fig2, ax2 = plot_cov_tube_3D(zkstar[:,3:5], Skstar[:, 3:5, 3:5], tf, 'red', 20, fig2, ax2)

    # ax2.set_title(r'Covariance $v_x-v_z$', fontdict=font)
    ax2.set_xlabel(r'Time $t$', fontdict=font)
    ax2.set_ylabel(r'$v_x(t)$', fontdict=font)
    ax2.set_zlabel(r'$v_z(t)$', fontdict=font)
    ax2.grid(False)
    ax2.legend()

    fig2.tight_layout()
    fig2.savefig(exp_script_dir+"/figures/pquad_cov_vxvz.pdf", transparent=True, dpi=1100, bbox_inches='tight', pad_inches=0.5)

    # ============================= 
    # covariance v\phi-\phi
    # =============================
    fig3 = plt.figure(frameon=False)
    ax3 = fig3.add_subplot(111, projection='3d')

    zkstar_angles = np.zeros((nt, 2))
    zkstar_angles[:, 0] = zkstar[:, 2]
    zkstar_angles[:, 1] = zkstar[:, 5]

    Skstar_angles = np.zeros((nt, 2, 2))
    Skstar_angles[:,0,0] = Skstar[:,0,0]
    Skstar_angles[:,0,1] = Skstar[:,2,5]
    Skstar_angles[:,1,0] = Skstar[:,5,2]
    Skstar_angles[:,1,1] = Skstar[:,5,5]

    fig3, ax3 = plot_cov_tube_3D(zkstar_angles[:, 0:2], Skstar_angles[:, 0:2, 0:2], tf, 'red', 20, fig3, ax3)

    # ax3.set_title(r'Covariance $\phi(t)-\dot \phi(t)$', fontdict=font)
    ax3.set_xlabel(r'Time $t$', fontdict=font)
    ax3.set_ylabel(r'$\phi(t)$', fontdict=font)
    ax3.set_zlabel(r'$\dot \phi(t)$', fontdict=font)

    for item in [fig3, ax3]:
        item.patch.set_visible(False)

    ax3.grid(False)    
    ax3.legend()

    fig3.tight_layout()
    fig3.savefig(exp_script_dir+"/figures/pquad_cov_vphiphi.pdf", transparent=True, bbox_inches='tight', pad_inches=0.5, dpi=1100)

    # -----------------
    # Plot state space 
    # ----------------- 
    fig1, ax1 = plt.subplots(1, 1,frameon=False)    

    # ax1.set_title(r'State space mean trajectory', fontdict=font)  
    ts = np.arange(0.0, tf+dt, dt)    
    ax1.grid(True)

    ax1.plot(ts[i], zkstar[i,0], linewidth=2.0, label=r'$p_x(t)$')
    ax1.plot(ts[i], zkstar[i,1], linewidth=2.0, label=r'$p_z(t)$')
    ax1.plot(ts[i], zkstar[i,2], linewidth=2.0, label=r'$\phi(t)$')
    ax1.plot(ts[i], zkstar[i,3], linewidth=2.0, label=r'$v_x(t)$')
    ax1.plot(ts[i], zkstar[i,4], linewidth=2.0, label=r'$v_z(t)$')
    ax1.plot(ts[i], zkstar[i,5], linewidth=2.0, label=r'$\dot \phi(t)$')

    # ===========
    # start states
    # ===========
    ax1.scatter(ts[0], x0[0], color='red', s=80, marker='x')
    ax1.scatter(ts[0], x0[1], color='red', s=80, marker='x')
    ax1.scatter(ts[0], x0[2], color='red', s=80, marker='x')
    ax1.scatter(ts[0], x0[3], color='red', s=80, marker='x')
    ax1.scatter(ts[0], x0[4], color='red', s=80, marker='x')
    ax1.scatter(ts[0], x0[5], color='red', s=80, marker='x', label=r'Start States')

    # ===========
    # goal states
    # ===========
    ax1.scatter(ts[nt-1], xT[0], color='green', s=80, marker='x')
    ax1.scatter(ts[nt-1], xT[1], color='green', s=80, marker='x')
    ax1.scatter(ts[nt-1], xT[2], color='green', s=80, marker='x')
    ax1.scatter(ts[nt-1], xT[3], color='green', s=80, marker='x')
    ax1.scatter(ts[nt-1], xT[4], color='green', s=80, marker='x')
    ax1.scatter(ts[nt-1], xT[5], color='green', s=80, marker='x', label=r'Goal States')

    ax1.set_xlabel(r'Time $t$', fontdict=font)
    ax1.set_ylabel(r'Values', fontdict=font)
    ax1.legend(loc='best')
    fig1.tight_layout()

    fig1.savefig(exp_script_dir+"/figures/pquad_mean_state.pdf", transparent=True, bbox_inches='tight', dpi=1100, pad_inches=0)

    plt.show()
    
    