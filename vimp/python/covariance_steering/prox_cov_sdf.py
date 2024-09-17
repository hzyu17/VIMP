from covariance_steering.compute_qr import *
from covariance_steering.compute_qr_pquad import *
from covariance_steering.linear_cov import *
from covariance_steering.pcs_data import *
from tools.propagations import *


file_path = os.path.abspath(__file__)
script_filename = os.path.splitext(os.path.basename(file_path))[0]
cur_dir = os.path.dirname(file_path)
py_dir = os.path.abspath(os.path.join(cur_dir, '..'))
debug_dir = os.path.abspath(os.path.join(py_dir, 'debug'))
    
import numpy as np
from tools.logger import *
from covariance_steering.pcs_data import *
from dynamics.planar_quad import *    

from tools.draw_pquadsdf_trj import *
from sdf_robot.scripts.generate_sdf_2d import *
import matplotlib.pyplot as plt

def proximal_cov_pquadsdf_zkSk(linearize_trj, 
                                zk, Sk, 
                                As, Bs, as_, 
                                Qt, rt, 
                                eps_obs, slope, sig_obs, radius,
                                epsilon, eta, iterations, iterations_linesearch,
                                x0, Sig0, xT, SigT, tf, mapname="SingleObstacleMap"):
    nt, nx, _ = As.shape
    nu = Bs.shape[2]
    
    show_iterations = True
    
    # ## ====================================== logging ======================================  
    # logger = Logger(nt, tf, epsilon, eta)
    if show_iterations:
        plt.ion()  # Turn on interactive plotting mode
        fig, ax = plt.subplots()
        _, planarmap = generate_2dsdf("SingleObstacleMap", savemap=False)
    
    ## ====================================== end logging ======================================
    pquadsdf = PQuadColSDF(eps_obs, slope, sig_obs, radius, map_name=mapname)
    
    exp_data_io = PCSIterationData()
    col_params = PquadCollisionParams(eps_obs, slope, sig_obs, radius, mapname)
    pcs_params = PCSParams(nt, tf, x0, xT, Sig0, SigT, epsilon, eta)
    exp_data_io.add_map_param(col_params)
    exp_data_io.add_pcs_param(pcs_params)
    
    
    # ======= For line search =======
    cur_total_cost = np.inf
    eta_base = eta
    
    hAk_prev = np.zeros((nt, nx, nx))
    hak_prev = np.zeros((nt, nx))
    
    iter_linearization = 0
    
    while True:
        print(" ========== Linearization iteration: ", str(iter_linearization), " ==========")
        
        # ------------------- linearization -------------------
        hAk, hBk, hak, nTr = linearize_trj(Sk, zk, As)
        iter_linearization += 1
        
        # -------------
        # convergence
        # -------------
        if np.linalg.norm(hAk-hAk_prev) < 1e-5 and np.linalg.norm(hak-hak_prev) < 1e-5:
            print("Converged.")
            break
        
        # ----------
        # algorithm
        # ----------
        for k in range(iterations):
            print(" ======= Optimizing iteration: ", str(k), " =======")
            
            # ========================
            # Plot the updated data
            # ========================
            if show_iterations:
                # Clear the current plot
                ax.clear()
                
                L = 5.0
                H = 0.35
                n_balls = 5
                # fig, ax = draw_pquad_trj_2d(x_trj_k, L, H, n_balls, fig, ax, step = 2000, draw_ball=False, save_fig=False)
                
                fig, ax = draw_pquad_map_trj_2d(x0, xT, zk, 
                                                L, H, n_balls, 
                                                fig, ax, planarmap, 
                                                step = 20, 
                                                draw_ball=False, save_fig=False, file_name="pquad_trj2d.pdf")
                
                ax.set_title(f"Iteration {k+1}")
                plt.draw()
                plt.pause(0.1)  # Pause briefly so changes are visible
            
            # ================ / end plotting ================
                    
            k_linesearch = 1
            eta = eta_base
            
            # ================================================ 
            #                   Line search
            # ================================================ 
            continue_iterations = True
            if k == 1:
                cur_total_cost = np.inf
            
            while (continue_iterations):
                k_linesearch += 1
                As_new = np.zeros_like(As)
                as_new = np.zeros_like(as_)
                
                print("Line search iteration ", k_linesearch)
                eta = eta*0.9
                
                Qk, rk, col_costs, gradient_col_states_nt = compute_qr_pquad(As, as_, hAk, hak, nTr, eta, Bs, Qt, rt, zk, pquadsdf)
                
                Aprior = (eta * As + hAk) / (1 + eta)
                aprior = (eta * as_ + hak) / (1 + eta)
                
                K, d, _, _ = linear_covcontrol(Aprior, Bs, aprior, epsilon, Qk, rk, x0, Sig0, xT, SigT, tf)
                
                for i in range(nt):
                    As_new[i] = Aprior[i] + hBk[i] @ K[i]
                    as_new[i] = aprior[i] + hBk[i] @ d[i]
                    
                zk_new, Sk_new = mean_cov_cl(As_new, Bs, as_new, epsilon, x0, Sig0, tf)
                ut = compute_control_signal(x0, K, d)
                
                control_costs = np.linalg.norm(ut) / 2.0
                new_total_cost = control_costs + col_costs
                
                print("New collision cost: ", col_costs)
                print("New control cost: ", control_costs)
                print("New total cost: ", new_total_cost)
                
                # -------------
                # update cost
                # -------------
                if (new_total_cost < cur_total_cost):
                    zk = zk_new
                    Sk = Sk_new
                    As = As_new
                    as_ = as_new
                    cur_total_cost = new_total_cost
                    
                    break
                
                # --------------------------
                # Line search reach limit 
                # --------------------------
                if (k_linesearch==iterations_linesearch):
                    continue_iterations = False
                    hAk_prev, hak_prev = hAk, hak
                    break
            
            if not (continue_iterations):
                break
            
            # ## ================================== logging ==================================
            data = PCSData(As, as_, hAk, hak, hBk, Qk, rk, zk, Sk, K, d)
            exp_data_io.add_iteration_data(k, data)
            
            # if (np.linalg.norm(As_prev.reshape((nt,nx*nx)) - As.reshape((nt,nx*nx))) < 1e-4) and (np.linalg.norm((as_prev - as_)) < 1e-4):
            #     print("Converged.")
            #     break

            # As_prev = As
            # as_prev = as_
            
            # logger.log_wandb()   
            # ## ================================== end logging ==================================
    
    
    plt.ioff()  # Turn off interactive plotting mode
    plt.show()  # Keep the window open

    # shutdown wandb
    # logger.shutdown_wandb()
    # ================================== save data ==================================
    from datetime import datetime
    current_datetime = datetime.now()
    formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"data_{formatted_datetime}_{script_filename}.pickle"
    exp_data_io.dump(f"{debug_dir}/PCS/{filename}")
    
    return As, as_, zk, Sk, gradient_col_states_nt


def proximal_cov_pquadsdf_zkSkKd(linearize_trj, 
                                zk, Sk, 
                                As, Bs, as_, 
                                Qt, rt, 
                                eps_obs, slope, sig_obs, radius,
                                epsilon, eta, iterations, iterations_linesearch,
                                x0, Sig0, xT, SigT, tf, mapname="SingleObstacleMap", xr=None):
        
    As, as_, zk, Sk, gradient_col_states_nt = proximal_cov_pquadsdf_zkSk(linearize_trj, 
                                                                        zk, Sk, 
                                                                        As, Bs, as_, 
                                                                        Qt, rt, 
                                                                        eps_obs, slope, sig_obs, radius,
                                                                        epsilon, eta, iterations, iterations_linesearch,
                                                                        x0, Sig0, xT, SigT, tf, mapname)
    
    # Recover optimal control
    zkstar, Skstar = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    hAstar, hBstar, hastar, nTr = linearize_trj(Skstar, zkstar, As)

    # Costs for recovering optimal control
    Qstar = Qt
    rstar = nTr / 2.0 + gradient_col_states_nt
    if xr is not None:
        for i_r in range(Qt.shape[0]):
            rstar[i_r] = rstar[i_r] -  Qt[i_r]@xr[i_r]
            
    Ks, ds, Pi_star, lbd_star = linear_covcontrol(hAstar, hBstar, hastar, epsilon, Qstar, rstar, x0, Sig0, xT, SigT, tf)
    
    return As, as_, zkstar, Skstar, Ks, ds, Pi_star, lbd_star

def proximal_cov_pquadsdf(linearize_pt, linearize_trj, 
                            Qt, rt, 
                            eps_obs, slope, sig_obs, radius,
                            epsilon, eta, iterations, iterations_linesearch,
                            x0, Sig0, xT, SigT, tf, nt):
        
    nx = x0.shape[0]
    
    _, hB1, _ = linearize_pt(x0)

    nu = hB1.shape[1]
    
    As = np.zeros((nt, nx, nx), dtype=np.float64)
    Bs = np.zeros((nt, nx, nu), dtype=np.float64)
    as_ = np.zeros((nt, nx), dtype=np.float64)
    
    for i in range(nt):
        Bs[i] = hB1
    
    zk, Sk = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    
    As, as_, zk, Sk, gradient_col_states_nt = proximal_cov_pquadsdf_zkSk(linearize_trj, zk, Sk, 
                                                                         As, Bs, as_, Qt, rt, 
                                                                        eps_obs, slope, sig_obs, radius, 
                                                                        epsilon, eta, iterations, iterations_linesearch,
                                                                        x0, Sig0, xT, SigT, tf)
    
    # Recover optimal control
    zkstar, Skstar = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    hAstar, hBstar, hastar, nTr = linearize_trj(Skstar, zkstar, As)

    # Costs for recovering optimal control
    Qstar = Qt
    rstar = nTr / 2.0 + gradient_col_states_nt
    Ks, ds, Pi_star, lbd_star = linear_covcontrol(hAstar, hBstar, hastar, epsilon, Qstar, rstar, x0, Sig0, xT, SigT, tf)
    
    return As, as_, Ks, ds, Pi_star, lbd_star


