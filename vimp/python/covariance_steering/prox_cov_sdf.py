"""
Proximal Covariance Steering for Planar Quadrotor with SDF-based Collision Avoidance.

This module implements iterative covariance steering algorithms with obstacle avoidance
using signed distance fields (SDF) for planar quadrotor systems.
"""

import os
from datetime import datetime
from dataclasses import dataclass
from typing import Tuple, Optional, Callable

import numpy as np
import matplotlib.pyplot as plt

# Local imports
from covariance_steering.compute_qr import *
from covariance_steering.compute_qr_pquad import *
from covariance_steering.linear_cov import *
from covariance_steering.pcs_data import *
from tools.propagations import *
from tools.logger import *
from tools.draw_pquadsdf_trj import *
from dynamics.planar_quad import *
from sdf_robot.scripts.generate_sdf_2d import *

# ============================================================================
# Path Configuration
# ============================================================================
_FILE_PATH = os.path.abspath(__file__)
_SCRIPT_NAME = os.path.splitext(os.path.basename(_FILE_PATH))[0]
_CUR_DIR = os.path.dirname(_FILE_PATH)
_PY_DIR = os.path.abspath(os.path.join(_CUR_DIR, '..'))
DEBUG_DIR = os.path.abspath(os.path.join(_PY_DIR, 'debug'))

# ============================================================================
# Constants
# ============================================================================
# Planar quadrotor geometry
PQUAD_LENGTH = 5.0
PQUAD_HEIGHT = 0.35
PQUAD_NUM_BALLS = 5

# Convergence thresholds
LINEARIZATION_CONVERGENCE_TOL = 1e-5

# Line search parameters
LINE_SEARCH_DECAY = 0.9


# ============================================================================
# Visualization Helpers
# ============================================================================
@dataclass
class VisualizationContext:
    """Context for iterative visualization during optimization."""
    fig: plt.Figure
    ax: plt.Axes
    planar_map: object
    
    @classmethod
    def create(cls, map_name: str) -> 'VisualizationContext':
        """Initialize visualization context with interactive plotting."""
        plt.ion()
        fig, ax = plt.subplots()
        _, planar_map = generate_2dsdf(map_name, savemap=False)
        return cls(fig=fig, ax=ax, planar_map=planar_map)
    
    def update(self, x0: np.ndarray, xT: np.ndarray, mean_trajectory: np.ndarray, 
               iteration: int, plot_step: int = 20) -> None:
        """Update visualization with current trajectory."""
        self.ax.clear()
        draw_pquad_map_trj_2d(
            x0, xT, mean_trajectory,
            PQUAD_LENGTH, PQUAD_HEIGHT, PQUAD_NUM_BALLS,
            self.fig, self.ax, self.planar_map,
            step=plot_step,
            draw_ball=False, save_fig=False,
            file_name="pquad_trj2d.pdf"
        )
        self.ax.set_title(f"Iteration {iteration + 1}")
        plt.draw()
        plt.pause(0.1)
    
    def finalize(self) -> None:
        """Finalize visualization and display final result."""
        plt.ioff()
        plt.show()


def save_experiment_data(exp_data: PCSIterationData, script_name: str) -> str:
    """Save experiment data to a timestamped pickle file."""
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"data_{timestamp}_{script_name}.pickle"
    filepath = os.path.join(DEBUG_DIR, "PCS", filename)
    exp_data.dump(filepath)
    return filepath


# ============================================================================
# Core Algorithm
# ============================================================================
def _check_linearization_convergence(
    hA_curr: np.ndarray, ha_curr: np.ndarray,
    hA_prev: np.ndarray, ha_prev: np.ndarray,
    tol: float = LINEARIZATION_CONVERGENCE_TOL
) -> bool:
    """Check if linearization has converged."""
    hA_converged = np.linalg.norm(hA_curr - hA_prev) < tol
    ha_converged = np.linalg.norm(ha_curr - ha_prev) < tol
    return hA_converged and ha_converged


def _perform_line_search_iteration(
    # Current state
    As: np.ndarray, as_: np.ndarray, Bs: np.ndarray,
    mean_trj: np.ndarray,
    # Linearization results  
    hA: np.ndarray, hB: np.ndarray, ha: np.ndarray, nominal_trace: np.ndarray,
    # Cost matrices
    Qt: np.ndarray, rt: np.ndarray,
    # Algorithm parameters
    eta: float, epsilon: float,
    # Boundary conditions
    x0: np.ndarray, Sig0: np.ndarray, xT: np.ndarray, SigT: np.ndarray, tf: float,
    # Collision model
    pquad_sdf: PQuadColSDF
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
    """
    Perform one iteration of the line search optimization.
    
    Returns:
        Tuple containing:
        - As_new: Updated closed-loop dynamics matrices
        - as_new: Updated closed-loop affine terms
        - mean_trj_new: Updated mean trajectory
        - cov_trj_new: Updated covariance trajectory
        - gradient_col: Collision cost gradients
        - collision_cost: Total collision cost
        - control_cost: Total control cost
    """
    nt, nx, _ = As.shape
    
    # Compute cost matrices with collision terms
    Qk, rk, collision_cost, gradient_col = compute_qr_pquad(
        As, as_, hA, ha, nominal_trace, eta, Bs, Qt, rt, mean_trj, pquad_sdf
    )
    
    # Compute proximal dynamics (blend current and linearized)
    A_proximal = (eta * As + hA) / (1 + eta)
    a_proximal = (eta * as_ + ha) / (1 + eta)
    
    # Solve covariance steering problem
    K, d, _, _ = linear_covcontrol(
        A_proximal, Bs, a_proximal, epsilon, Qk, rk, x0, Sig0, xT, SigT, tf
    )
    
    # Update closed-loop dynamics
    As_new = np.zeros_like(As)
    as_new = np.zeros_like(as_)
    for i in range(nt):
        As_new[i] = A_proximal[i] + hB[i] @ K[i]
        as_new[i] = a_proximal[i] + hB[i] @ d[i]
    
    # Propagate mean and covariance
    mean_trj_new, cov_trj_new = mean_cov_cl(As_new, Bs, as_new, epsilon, x0, Sig0, tf)
    
    # Compute control cost
    control_signal = compute_control_signal(x0, K, d)
    control_cost = np.linalg.norm(control_signal) / 2.0
    
    return As_new, as_new, mean_trj_new, cov_trj_new, gradient_col, collision_cost, control_cost


def proximal_cov_pquadsdf_zkSk(
    linearize_trj: Callable,
    mean_trj: np.ndarray,
    cov_trj: np.ndarray,
    As: np.ndarray,
    Bs: np.ndarray,
    as_: np.ndarray,
    Qt: np.ndarray,
    rt: np.ndarray,
    eps_obs: float,
    slope: float,
    sig_obs: float,
    radius: float,
    epsilon: float,
    eta: float,
    max_iterations: int,
    max_linesearch_iterations: int,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    map_name: str = "SingleObstacleMap",
    show_iterations: bool = True
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Proximal covariance steering with SDF collision avoidance.
    
    Solves the covariance steering problem for a planar quadrotor with obstacle
    avoidance using signed distance fields. Uses an iterative linearization
    approach with line search for convergence.
    
    Args:
        linearize_trj: Function to linearize dynamics along trajectory
        mean_trj: Initial mean trajectory (nt, nx)
        cov_trj: Initial covariance trajectory (nt, nx, nx)
        As: Closed-loop state transition matrices (nt, nx, nx)
        Bs: Control input matrices (nt, nx, nu)
        as_: Closed-loop affine terms (nt, nx)
        Qt: State cost matrices (nt, nx, nx)
        rt: State cost linear terms (nt, nx)
        eps_obs: Obstacle avoidance epsilon parameter
        slope: SDF slope parameter
        sig_obs: Obstacle sigma parameter
        radius: Robot collision radius
        epsilon: Covariance steering regularization
        eta: Proximal parameter
        max_iterations: Maximum optimization iterations per linearization
        max_linesearch_iterations: Maximum line search iterations
        x0: Initial state mean
        Sig0: Initial state covariance
        xT: Target state mean
        SigT: Target state covariance
        tf: Final time
        map_name: Name of the obstacle map
        show_iterations: Whether to visualize iterations
        
    Returns:
        Tuple containing:
        - As: Final closed-loop dynamics matrices
        - as_: Final closed-loop affine terms
        - mean_trj: Final mean trajectory
        - cov_trj: Final covariance trajectory
        - gradient_col: Final collision cost gradients
    """
    nt, nx, _ = As.shape
    
    # Initialize collision model
    pquad_sdf = PQuadColSDF(eps_obs, slope, sig_obs, radius, map_name=map_name)
    
    # Initialize experiment data logging
    exp_data = PCSIterationData()
    exp_data.add_map_param(PquadCollisionParams(eps_obs, slope, sig_obs, radius, map_name))
    exp_data.add_pcs_param(PCSParams(nt, tf, x0, xT, Sig0, SigT, epsilon, eta))
    
    # Initialize visualization if enabled
    vis_ctx = VisualizationContext.create(map_name) if show_iterations else None
    
    # Initialize tracking variables
    hA_prev = np.zeros((nt, nx, nx))
    ha_prev = np.zeros((nt, nx))
    current_cost = np.inf
    eta_base = eta
    gradient_col = None
    
    # Main linearization loop
    linearization_iter = 0
    while True:
        print(f" ========== Linearization iteration: {linearization_iter} ==========")
        
        # Linearize dynamics along current trajectory
        hA, hB, ha, nominal_trace = linearize_trj(cov_trj, mean_trj, As)
        linearization_iter += 1
        
        # Check convergence
        if _check_linearization_convergence(hA, ha, hA_prev, ha_prev):
            print("Converged.")
            break
        
        # Optimization iterations within current linearization
        for opt_iter in range(max_iterations):
            print(f" ======= Optimization iteration: {opt_iter} =======")
            
            # Update visualization
            if vis_ctx is not None:
                vis_ctx.update(x0, xT, mean_trj, opt_iter)
            
            # Reset line search parameters
            eta = eta_base
            if opt_iter == 1:
                current_cost = np.inf
            
            # Line search loop
            converged_linesearch = False
            for ls_iter in range(1, max_linesearch_iterations + 1):
                print(f"Line search iteration {ls_iter}")
                eta *= LINE_SEARCH_DECAY
                
                # Perform optimization step
                As_new, as_new, mean_new, cov_new, gradient_col, col_cost, ctrl_cost = \
                    _perform_line_search_iteration(
                        As, as_, Bs, mean_trj,
                        hA, hB, ha, nominal_trace,
                        Qt, rt, eta, epsilon,
                        x0, Sig0, xT, SigT, tf,
                        pquad_sdf
                    )
                
                new_cost = col_cost + ctrl_cost
                print(f"Collision cost: {col_cost:.6f}")
                print(f"Control cost: {ctrl_cost:.6f}")
                print(f"Total cost: {new_cost:.6f}")
                
                # Accept step if cost decreased
                if new_cost < current_cost:
                    mean_trj, cov_trj = mean_new, cov_new
                    As, as_ = As_new, as_new
                    current_cost = new_cost
                    converged_linesearch = True
                    break
            
            # Exit optimization if line search failed
            if not converged_linesearch:
                hA_prev, ha_prev = hA, ha
                break
            
            # Log iteration data
            Qk, rk, _, _ = compute_qr_pquad(
                As, as_, hA, ha, nominal_trace, eta, Bs, Qt, rt, mean_trj, pquad_sdf
            )
            K, d, _, _ = linear_covcontrol(
                (eta * As + hA) / (1 + eta), Bs, (eta * as_ + ha) / (1 + eta),
                epsilon, Qk, rk, x0, Sig0, xT, SigT, tf
            )
            exp_data.add_iteration_data(opt_iter, PCSData(As, as_, hA, ha, hB, Qk, rk, mean_trj, cov_trj, K, d))
        
        if not converged_linesearch:
            break
    
    # Finalize visualization
    if vis_ctx is not None:
        vis_ctx.finalize()
    
    # Save experiment data
    save_experiment_data(exp_data, _SCRIPT_NAME)
    
    return As, as_, mean_trj, cov_trj, gradient_col


def proximal_cov_pquadsdf_zkSkKd(
    linearize_trj: Callable,
    mean_trj: np.ndarray,
    cov_trj: np.ndarray,
    As: np.ndarray,
    Bs: np.ndarray,
    as_: np.ndarray,
    Qt: np.ndarray,
    rt: np.ndarray,
    eps_obs: float,
    slope: float,
    sig_obs: float,
    radius: float,
    epsilon: float,
    eta: float,
    max_iterations: int,
    max_linesearch_iterations: int,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    map_name: str = "SingleObstacleMap",
    xr: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Proximal covariance steering with optimal feedback gain recovery.
    
    Extends proximal_cov_pquadsdf_zkSk by recovering the optimal feedback gains
    K and d, along with the costate matrices Pi and lambda.
    
    Args:
        linearize_trj: Function to linearize dynamics along trajectory
        mean_trj: Initial mean trajectory
        cov_trj: Initial covariance trajectory
        As, Bs, as_: System dynamics
        Qt, rt: Cost matrices
        eps_obs, slope, sig_obs, radius: Collision parameters
        epsilon, eta: Algorithm parameters
        max_iterations, max_linesearch_iterations: Iteration limits
        x0, Sig0, xT, SigT, tf: Boundary conditions
        map_name: Obstacle map name
        xr: Reference trajectory for tracking (optional)
        
    Returns:
        Tuple containing:
        - As, as_: Final dynamics
        - mean_star, cov_star: Optimal trajectories
        - K, d: Optimal feedback gains
        - Pi_star, lambda_star: Costate matrices
    """
    # Run main optimization
    As, as_, mean_trj, cov_trj, gradient_col = proximal_cov_pquadsdf_zkSk(
        linearize_trj, mean_trj, cov_trj,
        As, Bs, as_, Qt, rt,
        eps_obs, slope, sig_obs, radius,
        epsilon, eta, max_iterations, max_linesearch_iterations,
        x0, Sig0, xT, SigT, tf, map_name
    )
    
    # Recover optimal trajectory and linearization
    mean_star, cov_star = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    hA_star, hB_star, ha_star, nominal_trace = linearize_trj(cov_star, mean_star, As)
    
    # Compute cost matrices for optimal control recovery
    Q_star = Qt
    r_star = nominal_trace / 2.0 + gradient_col
    
    # Add reference tracking term if provided
    if xr is not None:
        for i in range(Qt.shape[0]):
            r_star[i] = r_star[i] - Qt[i] @ xr[i]
    
    # Solve for optimal feedback gains
    K, d, Pi_star, lambda_star = linear_covcontrol(
        hA_star, hB_star, ha_star, epsilon, Q_star, r_star, x0, Sig0, xT, SigT, tf
    )
    
    return As, as_, mean_star, cov_star, K, d, Pi_star, lambda_star


def proximal_cov_pquadsdf(
    linearize_pt: Callable,
    linearize_trj: Callable,
    Qt: np.ndarray,
    rt: np.ndarray,
    eps_obs: float,
    slope: float,
    sig_obs: float,
    radius: float,
    epsilon: float,
    eta: float,
    max_iterations: int,
    max_linesearch_iterations: int,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    nt: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Proximal covariance steering from initial conditions.
    
    Initializes the problem from scratch using only boundary conditions,
    then solves for optimal trajectories and feedback gains.
    
    Args:
        linearize_pt: Function to linearize dynamics at a point
        linearize_trj: Function to linearize dynamics along trajectory
        Qt, rt: Cost matrices
        eps_obs, slope, sig_obs, radius: Collision parameters
        epsilon, eta: Algorithm parameters
        max_iterations, max_linesearch_iterations: Iteration limits
        x0, Sig0, xT, SigT, tf: Boundary conditions
        nt: Number of time steps
        
    Returns:
        Tuple containing:
        - As, as_: Final dynamics
        - K, d: Optimal feedback gains
        - Pi_star, lambda_star: Costate matrices
    """
    nx = x0.shape[0]
    
    # Get control input dimension from linearization
    _, hB_init, _ = linearize_pt(x0)
    nu = hB_init.shape[1]
    
    # Initialize dynamics arrays
    As = np.zeros((nt, nx, nx), dtype=np.float64)
    Bs = np.zeros((nt, nx, nu), dtype=np.float64)
    as_ = np.zeros((nt, nx), dtype=np.float64)
    
    # Set constant input matrix
    for i in range(nt):
        Bs[i] = hB_init
    
    # Initialize trajectory from open-loop propagation
    mean_init, cov_init = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    
    # Run main optimization
    As, as_, mean_trj, cov_trj, gradient_col = proximal_cov_pquadsdf_zkSk(
        linearize_trj, mean_init, cov_init,
        As, Bs, as_, Qt, rt,
        eps_obs, slope, sig_obs, radius,
        epsilon, eta, max_iterations, max_linesearch_iterations,
        x0, Sig0, xT, SigT, tf
    )
    
    # Recover optimal trajectory and linearization
    mean_star, cov_star = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    hA_star, hB_star, ha_star, nominal_trace = linearize_trj(cov_star, mean_star, As)
    
    # Compute cost matrices for optimal control recovery
    Q_star = Qt
    r_star = nominal_trace / 2.0 + gradient_col
    
    # Solve for optimal feedback gains
    K, d, Pi_star, lambda_star = linear_covcontrol(
        hA_star, hB_star, ha_star, epsilon, Q_star, r_star, x0, Sig0, xT, SigT, tf
    )
    
    return As, as_, K, d, Pi_star, lambda_star
