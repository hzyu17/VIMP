"""
Proximal Covariance Steering for 3D Quadrotor with SDF-based Collision Avoidance.

This module implements iterative covariance steering algorithms with obstacle avoidance
using signed distance fields (SDF) for 3D quadrotor systems.

State: [x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r] (12D)
Control: [T, τx, τy, τz] (4D)

Author: Adapted from planar quadrotor implementation
"""

import os
from datetime import datetime
from dataclasses import dataclass, field
from typing import Tuple, Optional, Callable, Dict, Any

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Local imports
from covariance_steering.linear_cov import linear_covcontrol
from tools.propagations import mean_cov_cl, compute_control_signal
from dynamics.quad3d import (
    Quad3DParams,
    quad3d_A_matrix,
    quad3d_B_matrix,
    quad3d_dynamics,
    linearize_quad3d,
    rotation_matrix_zyx,
)

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
# State and control dimensions
NX_3D = 12
NU_3D = 4

# Convergence thresholds
LINEARIZATION_CONVERGENCE_TOL = 1e-5

# Line search parameters
LINE_SEARCH_DECAY = 0.9

# Visualization
PLOT_UPDATE_INTERVAL = 0.1

# Numerical stability
MIN_EIGENVALUE = 1e-8
COVARIANCE_REGULARIZATION = 1e-6


# ============================================================================
# Data Classes
# ============================================================================
@dataclass
class Quad3DCollisionParams:
    """Parameters for 3D collision cost computation."""
    eps_obs: float          # Obstacle avoidance epsilon
    slope: float            # SDF slope parameter
    sig_obs: float          # Obstacle sigma parameter
    radius: float           # Robot collision radius
    map_name: str = "3DObstacleMap"
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'eps_obs': self.eps_obs,
            'slope': self.slope,
            'sig_obs': self.sig_obs,
            'radius': self.radius,
            'map_name': self.map_name,
        }


@dataclass
class PCS3DParams:
    """Parameters for PCS algorithm."""
    nt: int                 # Number of time steps
    tf: float               # Final time
    x0: np.ndarray          # Initial state mean
    xT: np.ndarray          # Target state mean
    Sig0: np.ndarray        # Initial state covariance
    SigT: np.ndarray        # Target state covariance
    epsilon: float          # Covariance steering regularization
    eta: float              # Proximal parameter
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'nt': self.nt,
            'tf': self.tf,
            'x0': self.x0.tolist(),
            'xT': self.xT.tolist(),
            'Sig0': self.Sig0.tolist(),
            'SigT': self.SigT.tolist(),
            'epsilon': self.epsilon,
            'eta': self.eta,
        }


@dataclass
class PCS3DIterationData:
    """Data from a single PCS iteration."""
    As: np.ndarray          # Closed-loop dynamics
    as_: np.ndarray         # Closed-loop affine terms
    hA: np.ndarray          # Linearized A matrices
    ha: np.ndarray          # Linearized affine terms
    hB: np.ndarray          # Linearized B matrices
    Q: np.ndarray           # Cost matrices
    r: np.ndarray           # Cost linear terms
    mean_trj: np.ndarray    # Mean trajectory
    cov_trj: np.ndarray     # Covariance trajectory
    K: np.ndarray           # Feedback gains
    d: np.ndarray           # Feedforward terms
    cost: float = 0.0       # Total cost


@dataclass
class PCS3DExperimentData:
    """Container for full experiment data."""
    collision_params: Optional[Quad3DCollisionParams] = None
    pcs_params: Optional[PCS3DParams] = None
    quad_params: Optional[Quad3DParams] = None
    iterations: Dict[int, PCS3DIterationData] = field(default_factory=dict)
    
    def add_iteration(self, idx: int, data: PCS3DIterationData) -> None:
        self.iterations[idx] = data
    
    def save(self, filepath: str) -> None:
        """Save experiment data to pickle file."""
        import pickle
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'wb') as f:
            pickle.dump(self, f)
    
    @classmethod
    def load(cls, filepath: str) -> 'PCS3DExperimentData':
        """Load experiment data from pickle file."""
        import pickle
        with open(filepath, 'rb') as f:
            return pickle.load(f)


# ============================================================================
# Numerical Utilities
# ============================================================================
def ensure_positive_definite(matrix: np.ndarray, min_eig: float = MIN_EIGENVALUE) -> np.ndarray:
    """
    Ensure matrix is positive definite by adjusting eigenvalues.
    
    Args:
        matrix: Input symmetric matrix
        min_eig: Minimum allowed eigenvalue
        
    Returns:
        Positive definite matrix
    """
    # Symmetrize
    matrix = (matrix + matrix.T) / 2
    
    # Eigendecomposition
    eigenvalues, eigenvectors = np.linalg.eigh(matrix)
    
    # Clip eigenvalues
    eigenvalues = np.maximum(eigenvalues, min_eig)
    
    # Reconstruct
    return eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T


def regularize_covariance(cov: np.ndarray, reg: float = COVARIANCE_REGULARIZATION) -> np.ndarray:
    """Add regularization to covariance matrix."""
    return cov + reg * np.eye(cov.shape[0])


def safe_matrix_sqrt(matrix: np.ndarray) -> np.ndarray:
    """Compute matrix square root with numerical safeguards."""
    matrix = ensure_positive_definite(matrix)
    eigenvalues, eigenvectors = np.linalg.eigh(matrix)
    eigenvalues = np.maximum(eigenvalues, MIN_EIGENVALUE)
    return eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T


def check_finite(arr: np.ndarray, name: str = "array") -> bool:
    """Check if array contains only finite values."""
    if not np.all(np.isfinite(arr)):
        print(f"Warning: {name} contains non-finite values")
        print(f"  NaN count: {np.sum(np.isnan(arr))}")
        print(f"  Inf count: {np.sum(np.isinf(arr))}")
        return False
    return True


def safe_matrix_solve(A: np.ndarray, b: np.ndarray, reg: float = 1e-6) -> np.ndarray:
    """
    Solve Ax = b with regularization for near-singular matrices.
    
    Args:
        A: Matrix (n x n)
        b: Right-hand side (n x m) or (n,)
        reg: Regularization parameter
        
    Returns:
        x: Solution
    """
    n = A.shape[0]
    try:
        # Try direct solve first
        return np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        # Add regularization and retry
        A_reg = A + reg * np.eye(n)
        try:
            return np.linalg.solve(A_reg, b)
        except np.linalg.LinAlgError:
            # Fall back to pseudoinverse
            print(f"Warning: Using pseudoinverse for singular matrix")
            return np.linalg.lstsq(A, b, rcond=None)[0]


# ============================================================================
# 3D Collision Cost (SDF-based)
# ============================================================================
class Quad3DCollisionSDF:
    """
    3D collision cost using signed distance fields.
    
    Computes collision costs and gradients for a 3D quadrotor represented
    as a collection of spheres (body + rotors).
    """
    
    def __init__(self, params: Quad3DCollisionParams, quad_params: Quad3DParams,
                 sdf_func: Optional[Callable] = None):
        """
        Initialize 3D collision model.
        
        Args:
            params: Collision parameters
            quad_params: Quadrotor physical parameters
            sdf_func: Signed distance function sdf(position) -> (distance, gradient)
                      If None, uses a default spherical obstacle
        """
        self.params = params
        self.quad_params = quad_params
        self.sdf_func = sdf_func or self._default_sdf
        
        # Precompute sphere offsets in body frame
        self._setup_collision_spheres()
    
    def _setup_collision_spheres(self) -> None:
        """Setup collision sphere positions in body frame."""
        L = self.quad_params.arm_length
        
        # Body center sphere
        self.sphere_offsets = [np.array([0, 0, 0])]
        self.sphere_radii = [self.quad_params.body_radius]
        
        # Rotor spheres (if enabled)
        if self.quad_params.use_rotor_spheres:
            rotor_positions = [
                np.array([L, 0, 0]),
                np.array([-L, 0, 0]),
                np.array([0, L, 0]),
                np.array([0, -L, 0]),
            ]
            for pos in rotor_positions:
                self.sphere_offsets.append(pos)
                self.sphere_radii.append(self.quad_params.rotor_radius)
    
    def _default_sdf(self, position: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Default SDF: single spherical obstacle at origin.
        
        Args:
            position: Query position [3]
            
        Returns:
            (signed_distance, gradient)
        """
        obstacle_center = np.array([5.0, 5.0, 5.0])
        obstacle_radius = 2.0
        
        diff = position - obstacle_center
        dist = np.linalg.norm(diff)
        
        if dist < 1e-6:
            return -obstacle_radius, np.zeros(3)
        
        signed_dist = dist - obstacle_radius
        gradient = diff / dist
        
        return signed_dist, gradient
    
    def get_sphere_positions_world(self, state: np.ndarray) -> np.ndarray:
        """
        Get world positions of all collision spheres.
        
        Args:
            state: Quadrotor state [12]
            
        Returns:
            positions: Sphere positions in world frame [n_spheres, 3]
        """
        position = state[0:3]
        phi, theta, psi = state[3], state[4], state[5]
        R = rotation_matrix_zyx(phi, theta, psi)
        
        positions = []
        for offset in self.sphere_offsets:
            world_pos = position + R @ offset
            positions.append(world_pos)
        
        return np.array(positions)
    
    def collision_cost(self, state: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Compute collision cost and gradient for a single state.
        
        Uses soft hinge loss: cost = sig * log(1 + exp(slope * (radius - sdf + eps)))
        
        Args:
            state: Quadrotor state [12]
            
        Returns:
            (cost, gradient_wrt_state)
        """
        eps = self.params.eps_obs
        slope = self.params.slope
        sig = self.params.sig_obs
        
        total_cost = 0.0
        gradient = np.zeros(NX_3D)
        
        position = state[0:3]
        phi, theta, psi = state[3], state[4], state[5]
        R = rotation_matrix_zyx(phi, theta, psi)
        
        # Import rotation derivatives
        from dynamics.quad3d import rotation_matrix_derivatives
        dR_dphi, dR_dtheta, dR_dpsi = rotation_matrix_derivatives(phi, theta, psi)
        
        for offset, radius in zip(self.sphere_offsets, self.sphere_radii):
            # Sphere position in world frame
            sphere_pos = position + R @ offset
            
            # Get SDF value and gradient
            sdf_val, sdf_grad = self.sdf_func(sphere_pos)
            
            # Soft hinge cost
            margin = radius + self.params.radius - sdf_val + eps
            exp_term = np.exp(np.clip(slope * margin, -50, 50))
            cost_i = sig * np.log(1 + exp_term)
            total_cost += cost_i
            
            # Gradient of cost w.r.t. sphere position
            dcost_dsdf = -sig * slope * exp_term / (1 + exp_term)
            dcost_dpos = dcost_dsdf * sdf_grad
            
            # Chain rule to state gradient
            # ∂cost/∂position = ∂cost/∂sphere_pos
            gradient[0:3] += dcost_dpos
            
            # ∂cost/∂euler = ∂cost/∂sphere_pos @ ∂sphere_pos/∂euler
            # sphere_pos = position + R @ offset
            # ∂sphere_pos/∂phi = dR_dphi @ offset
            gradient[3] += dcost_dpos @ (dR_dphi @ offset)
            gradient[4] += dcost_dpos @ (dR_dtheta @ offset)
            gradient[5] += dcost_dpos @ (dR_dpsi @ offset)
        
        return total_cost, gradient
    
    def trajectory_collision_cost(self, mean_trj: np.ndarray
                                   ) -> Tuple[float, np.ndarray]:
        """
        Compute total collision cost over trajectory.
        
        Args:
            mean_trj: Mean trajectory [nt, nx]
            
        Returns:
            (total_cost, gradients) where gradients is [nt, nx]
        """
        nt = mean_trj.shape[0]
        total_cost = 0.0
        gradients = np.zeros((nt, NX_3D))
        
        for i in range(nt):
            cost_i, grad_i = self.collision_cost(mean_trj[i])
            total_cost += cost_i
            gradients[i] = grad_i
        
        return total_cost, gradients


# ============================================================================
# 3D Visualization
# ============================================================================
@dataclass
class Visualization3DContext:
    """Context for 3D visualization during optimization."""
    fig: plt.Figure
    ax: Axes3D
    obstacles: list = field(default_factory=list)
    
    @classmethod
    def create(cls, obstacles: Optional[list] = None) -> 'Visualization3DContext':
        """Initialize 3D visualization context."""
        plt.ion()
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        return cls(fig=fig, ax=ax, obstacles=obstacles or [])
    
    def update(self, x0: np.ndarray, xT: np.ndarray, mean_trj: np.ndarray,
               iteration: int, quad_params: Quad3DParams) -> None:
        """Update 3D visualization with current trajectory."""
        self.ax.clear()
        
        # Plot trajectory
        self.ax.plot(mean_trj[:, 0], mean_trj[:, 1], mean_trj[:, 2],
                     'b-', linewidth=2, label='Trajectory')
        
        # Plot start and goal
        self.ax.scatter(*x0[0:3], c='green', s=100, marker='o', label='Start')
        self.ax.scatter(*xT[0:3], c='red', s=100, marker='*', label='Goal')
        
        # Plot quadrotor at selected timesteps
        n_frames = 20
        frame_indices = np.linspace(0, len(mean_trj) - 1, n_frames, dtype=int)
        
        for idx in frame_indices:
            self._draw_quadrotor(mean_trj[idx], quad_params, alpha=0.5)
        
        # Plot obstacles
        for obs in self.obstacles:
            self._draw_sphere(obs['center'], obs['radius'], color='red', alpha=0.3)
        
        # Labels and title
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.set_title(f'PCS Iteration {iteration + 1}')
        self.ax.legend()
        
        # Equal aspect ratio
        self._set_axes_equal()
        
        plt.draw()
        plt.pause(PLOT_UPDATE_INTERVAL)
    
    def _draw_quadrotor(self, state: np.ndarray, params: Quad3DParams,
                        alpha: float = 1.0) -> None:
        """Draw quadrotor frame at given state."""
        position = state[0:3]
        phi, theta, psi = state[3], state[4], state[5]
        R = rotation_matrix_zyx(phi, theta, psi)
        
        L = params.arm_length
        
        # Rotor positions in body frame
        rotor_body = [
            np.array([L, 0, 0]),
            np.array([-L, 0, 0]),
            np.array([0, L, 0]),
            np.array([0, -L, 0]),
        ]
        
        # Transform to world frame
        rotor_world = [position + R @ r for r in rotor_body]
        
        # Draw arms
        for rw in rotor_world:
            self.ax.plot([position[0], rw[0]],
                        [position[1], rw[1]],
                        [position[2], rw[2]],
                        'k-', linewidth=2, alpha=alpha)
        
        # Draw rotors as circles
        for rw in rotor_world:
            self.ax.scatter(*rw, c='blue', s=30, alpha=alpha)
    
    def _draw_sphere(self, center: np.ndarray, radius: float,
                     color: str = 'red', alpha: float = 0.3) -> None:
        """Draw a sphere obstacle."""
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        self.ax.plot_surface(x, y, z, color=color, alpha=alpha)
    
    def _set_axes_equal(self) -> None:
        """Set equal aspect ratio for 3D plot."""
        limits = np.array([
            self.ax.get_xlim3d(),
            self.ax.get_ylim3d(),
            self.ax.get_zlim3d(),
        ])
        
        center = np.mean(limits, axis=1)
        max_range = np.max(limits[:, 1] - limits[:, 0]) / 2
        
        self.ax.set_xlim3d([center[0] - max_range, center[0] + max_range])
        self.ax.set_ylim3d([center[1] - max_range, center[1] + max_range])
        self.ax.set_zlim3d([center[2] - max_range, center[2] + max_range])
    
    def finalize(self) -> None:
        """Finalize visualization."""
        plt.ioff()
        plt.show()


# ============================================================================
# Linearization Functions
# ============================================================================
def linearize_trajectory_3d(
    cov_trj: np.ndarray,
    mean_trj: np.ndarray,
    As: np.ndarray,
    quad_params: Quad3DParams,
    dt: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Linearize 3D quadrotor dynamics along trajectory.
    
    Args:
        cov_trj: Covariance trajectory [nt, nx, nx]
        mean_trj: Mean trajectory [nt, nx]
        As: Current closed-loop dynamics [nt, nx, nx]
        quad_params: Quadrotor parameters
        dt: Time step
        
    Returns:
        hA: Linearized A matrices [nt, nx, nx]
        hB: Linearized B matrices [nt, nx, nu]
        ha: Linearized affine terms [nt, nx]
        trace_terms: Trace terms for stochastic cost [nt, nx]
    """
    nt = mean_trj.shape[0]
    
    hA = np.zeros((nt, NX_3D, NX_3D))
    hB = np.zeros((nt, NX_3D, NU_3D))
    ha = np.zeros((nt, NX_3D))
    trace_terms = np.zeros((nt, NX_3D))
    
    for i in range(nt):
        # Nominal control: hover thrust
        u_nom = np.array([quad_params.mass * quad_params.g, 0, 0, 0])
        
        # Linearize at current state
        A_d, B_d, a_d = linearize_quad3d(mean_trj[i], u_nom, dt, quad_params)
        
        hA[i] = A_d
        hB[i] = B_d
        ha[i] = a_d
        
        # Trace term for expected cost under uncertainty
        # This captures the effect of covariance on expected costs
        trace_terms[i] = np.zeros(NX_3D)
    
    return hA, hB, ha, trace_terms


def linearize_point_3d(
    state: np.ndarray,
    quad_params: Quad3DParams,
    dt: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Linearize 3D quadrotor dynamics at a single point.
    
    Args:
        state: State vector [12]
        quad_params: Quadrotor parameters
        dt: Time step
        
    Returns:
        A: State matrix [nx, nx]
        B: Input matrix [nx, nu]
        a: Affine term [nx]
    """
    u_nom = np.array([quad_params.mass * quad_params.g, 0, 0, 0])
    return linearize_quad3d(state, u_nom, dt, quad_params)


# ============================================================================
# Trajectory Initialization
# ============================================================================
def initialize_trajectory_interpolation(
    x0: np.ndarray,
    xT: np.ndarray,
    nt: int,
    tf: float,
    quad_params: Quad3DParams
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Initialize trajectory with proper dynamics-aware interpolation.
    
    This creates a feasible initial trajectory by:
    1. Interpolating position linearly
    2. Setting velocities based on position derivatives
    3. Keeping orientation and angular rates near hover
    
    Args:
        x0: Initial state [12]
        xT: Target state [12]
        nt: Number of timesteps
        tf: Final time
        quad_params: Quadrotor parameters
        
    Returns:
        mean_trj: Initial mean trajectory [nt, 12]
        cov_trj: Initial covariance trajectory [nt, 12, 12]
        As: Initial closed-loop dynamics [nt, 12, 12]
        as_: Initial affine terms [nt, 12]
    """
    dt = tf / (nt - 1)
    
    # Initialize arrays
    mean_trj = np.zeros((nt, NX_3D))
    cov_trj = np.zeros((nt, NX_3D, NX_3D))
    As = np.zeros((nt, NX_3D, NX_3D))
    as_ = np.zeros((nt, NX_3D))
    
    # Position interpolation with smooth acceleration profile
    # Using quintic polynomial for smooth start/end
    for i in range(nt):
        t = i / (nt - 1)
        
        # Quintic interpolation: s(t) = 6t^5 - 15t^4 + 10t^3
        s = 6*t**5 - 15*t**4 + 10*t**3
        s_dot = (30*t**4 - 60*t**3 + 30*t**2) / tf
        
        # Position
        mean_trj[i, 0:3] = (1 - s) * x0[0:3] + s * xT[0:3]
        
        # Velocity (derivative of position interpolation)
        mean_trj[i, 6:9] = s_dot * (xT[0:3] - x0[0:3])
        
        # Orientation: interpolate smoothly
        mean_trj[i, 3:6] = (1 - s) * x0[3:6] + s * xT[3:6]
        
        # Angular rates: interpolate
        mean_trj[i, 9:12] = (1 - s) * x0[9:12] + s * xT[9:12]
        
        # Initial covariance (small, isotropic)
        cov_trj[i] = 0.01 * np.eye(NX_3D)
        
        # Initialize As with simple stable dynamics
        # Use linearization at current state
        A_lin, _, a_lin = linearize_point_3d(mean_trj[i], quad_params, dt)
        
        # Make slightly contractive for stability
        As[i] = 0.99 * A_lin
        as_[i] = a_lin
    
    return mean_trj, cov_trj, As, as_


def initialize_with_simulation(
    x0: np.ndarray,
    xT: np.ndarray,
    nt: int,
    tf: float,
    quad_params: Quad3DParams
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Initialize trajectory by simulating with simple PD control toward goal.
    
    Args:
        x0: Initial state
        xT: Target state
        nt: Number of timesteps
        tf: Final time
        quad_params: Quadrotor parameters
        
    Returns:
        mean_trj: Simulated trajectory
        u_trj: Control trajectory
    """
    from dynamics.quad3d import quad3d_dynamics
    
    dt = tf / (nt - 1)
    mean_trj = np.zeros((nt, NX_3D))
    u_trj = np.zeros((nt, NU_3D))
    
    mean_trj[0] = x0
    
    # Simple PD gains
    Kp_pos = 2.0
    Kd_pos = 1.0
    Kp_att = 5.0
    Kd_att = 1.0
    
    for i in range(nt - 1):
        state = mean_trj[i]
        
        # Position error
        pos_err = xT[0:3] - state[0:3]
        vel_err = xT[6:9] - state[6:9]
        
        # Desired acceleration
        acc_des = Kp_pos * pos_err + Kd_pos * vel_err
        
        # Compute required thrust and attitude
        # Simplified: assume small angles
        thrust = quad_params.mass * (quad_params.g + acc_des[2])
        thrust = np.clip(thrust, 0.1 * quad_params.mass * quad_params.g, 
                        2.0 * quad_params.mass * quad_params.g)
        
        # Attitude control
        att_err = xT[3:6] - state[3:6]
        rate_err = xT[9:12] - state[9:12]
        
        torques = Kp_att * att_err + Kd_att * rate_err
        torques = np.clip(torques, -1.0, 1.0)
        
        u = np.array([thrust, torques[0], torques[1], torques[2]])
        u_trj[i] = u
        
        # Integrate with RK4
        k1 = quad3d_dynamics(state, u, quad_params)
        k2 = quad3d_dynamics(state + 0.5*dt*k1, u, quad_params)
        k3 = quad3d_dynamics(state + 0.5*dt*k2, u, quad_params)
        k4 = quad3d_dynamics(state + dt*k3, u, quad_params)
        
        mean_trj[i+1] = state + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
    
    return mean_trj, u_trj


# ============================================================================
# Cost Computation
# ============================================================================
def compute_cost_matrices_3d(
    As: np.ndarray,
    as_: np.ndarray,
    hA: np.ndarray,
    ha: np.ndarray,
    trace_terms: np.ndarray,
    eta: float,
    Bs: np.ndarray,
    Qt: np.ndarray,
    rt: np.ndarray,
    mean_trj: np.ndarray,
    collision_model: Quad3DCollisionSDF
) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """
    Compute cost matrices including collision costs.
    
    Args:
        As: Current closed-loop dynamics [nt, nx, nx]
        as_: Current closed-loop affine terms [nt, nx]
        hA: Linearized A matrices [nt, nx, nx]
        ha: Linearized affine terms [nt, nx]
        trace_terms: Trace terms [nt, nx]
        eta: Proximal parameter
        Bs: Input matrices [nt, nx, nu]
        Qt: State cost matrices [nt, nx, nx]
        rt: State cost linear terms [nt, nx]
        mean_trj: Mean trajectory [nt, nx]
        collision_model: Collision cost model
        
    Returns:
        Q: Total cost matrices [nt, nx, nx]
        r: Total cost linear terms [nt, nx]
        collision_cost: Total collision cost
        collision_gradients: Collision gradients [nt, nx]
    """
    nt = As.shape[0]
    
    # Compute collision costs and gradients
    collision_cost, collision_gradients = collision_model.trajectory_collision_cost(mean_trj)
    
    # Build total cost matrices
    Q = np.copy(Qt)
    r = np.copy(rt)
    
    # Add collision gradient to linear cost
    for i in range(nt):
        r[i] = r[i] + collision_gradients[i]
        # Add trace terms
        r[i] = r[i] + trace_terms[i] / 2.0
        
        # Ensure Q is positive definite
        Q[i] = ensure_positive_definite(Q[i] + COVARIANCE_REGULARIZATION * np.eye(NX_3D))
    
    return Q, r, collision_cost, collision_gradients


# ============================================================================
# Core Algorithm Helpers
# ============================================================================
def _check_convergence(
    hA_curr: np.ndarray,
    ha_curr: np.ndarray,
    hA_prev: np.ndarray,
    ha_prev: np.ndarray,
    tol: float = LINEARIZATION_CONVERGENCE_TOL
) -> bool:
    """Check if linearization has converged."""
    return (np.linalg.norm(hA_curr - hA_prev) < tol and
            np.linalg.norm(ha_curr - ha_prev) < tol)


def _safe_linear_covcontrol(
    A: np.ndarray,
    B: np.ndarray,
    a: np.ndarray,
    epsilon: float,
    Q: np.ndarray,
    r: np.ndarray,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Wrapper for linear_covcontrol with numerical safeguards.
    
    Returns:
        K, d, Pi, lbd (or raises exception with diagnostics)
    """
    nt = A.shape[0]
    
    # Ensure positive definiteness of covariances
    Sig0_safe = ensure_positive_definite(Sig0)
    SigT_safe = ensure_positive_definite(SigT)
    
    # Check inputs
    for i in range(nt):
        if not check_finite(A[i], f"A[{i}]"):
            raise ValueError(f"Non-finite values in A[{i}]")
        if not check_finite(B[i], f"B[{i}]"):
            raise ValueError(f"Non-finite values in B[{i}]")
        if not check_finite(a[i], f"a[{i}]"):
            raise ValueError(f"Non-finite values in a[{i}]")
        if not check_finite(Q[i], f"Q[{i}]"):
            raise ValueError(f"Non-finite values in Q[{i}]")
        if not check_finite(r[i], f"r[{i}]"):
            raise ValueError(f"Non-finite values in r[{i}]")
        
        # Ensure Q is positive semi-definite
        Q[i] = ensure_positive_definite(Q[i], min_eig=0)
    
    try:
        K, d, Pi, lbd = linear_covcontrol(
            A, B, a, epsilon, Q, r, x0, Sig0_safe, xT, SigT_safe, tf
        )
        
        # Check outputs
        if not np.all(np.isfinite(K)):
            raise ValueError("Non-finite values in feedback gain K")
        if not np.all(np.isfinite(d)):
            raise ValueError("Non-finite values in feedforward term d")
            
        return K, d, Pi, lbd
        
    except Exception as e:
        print(f"linear_covcontrol failed: {e}")
        print(f"  Sig0 eigenvalues: {np.linalg.eigvalsh(Sig0_safe)}")
        print(f"  SigT eigenvalues: {np.linalg.eigvalsh(SigT_safe)}")
        print(f"  epsilon: {epsilon}")
        print(f"  tf: {tf}")
        raise


def _line_search_step(
    As: np.ndarray,
    as_: np.ndarray,
    Bs: np.ndarray,
    mean_trj: np.ndarray,
    hA: np.ndarray,
    hB: np.ndarray,
    ha: np.ndarray,
    trace_terms: np.ndarray,
    Qt: np.ndarray,
    rt: np.ndarray,
    eta: float,
    epsilon: float,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    collision_model: Quad3DCollisionSDF
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
    """
    Perform one line search iteration.
    
    Returns:
        As_new, as_new, mean_new, cov_new, collision_grad, collision_cost, control_cost
    """
    nt, nx, _ = As.shape
    
    # Compute cost matrices
    Q, r, collision_cost, collision_grad = compute_cost_matrices_3d(
        As, as_, hA, ha, trace_terms, eta, Bs, Qt, rt, mean_trj, collision_model
    )
    
    # Proximal dynamics
    A_prox = (eta * As + hA) / (1 + eta)
    a_prox = (eta * as_ + ha) / (1 + eta)
    
    # Solve covariance steering with safeguards
    K, d, _, _ = _safe_linear_covcontrol(
        A_prox, Bs, a_prox, epsilon, Q, r, x0, Sig0, xT, SigT, tf
    )
    
    # Update closed-loop dynamics
    As_new = np.zeros_like(As)
    as_new = np.zeros_like(as_)
    
    for i in range(nt):
        As_new[i] = A_prox[i] + hB[i] @ K[i]
        as_new[i] = a_prox[i] + hB[i] @ d[i]
    
    # Propagate
    mean_new, cov_new = mean_cov_cl(As_new, Bs, as_new, epsilon, x0, Sig0, tf)
    
    # Check for numerical issues
    if not np.all(np.isfinite(mean_new)):
        raise ValueError("Non-finite values in propagated mean trajectory")
    
    # Control cost
    control_signal = compute_control_signal(x0, K, d)
    control_cost = np.linalg.norm(control_signal) / 2.0
    
    return As_new, as_new, mean_new, cov_new, collision_grad, collision_cost, control_cost


# ============================================================================
# Main PCS Algorithm
# ============================================================================
def proximal_cov_steering_3d(
    mean_trj: np.ndarray,
    cov_trj: np.ndarray,
    As: np.ndarray,
    Bs: np.ndarray,
    as_: np.ndarray,
    Qt: np.ndarray,
    rt: np.ndarray,
    collision_params: Quad3DCollisionParams,
    quad_params: Quad3DParams,
    epsilon: float,
    eta: float,
    max_iterations: int,
    max_linesearch_iterations: int,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    sdf_func: Optional[Callable] = None,
    show_iterations: bool = True,
    obstacles: Optional[list] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Proximal Covariance Steering for 3D quadrotor with collision avoidance.
    
    Args:
        mean_trj: Initial mean trajectory [nt, 12]
        cov_trj: Initial covariance trajectory [nt, 12, 12]
        As: Closed-loop state matrices [nt, 12, 12]
        Bs: Control input matrices [nt, 12, 4]
        as_: Closed-loop affine terms [nt, 12]
        Qt: State cost matrices [nt, 12, 12]
        rt: State cost linear terms [nt, 12]
        collision_params: Collision avoidance parameters
        quad_params: Quadrotor physical parameters
        epsilon: Covariance steering regularization
        eta: Proximal parameter
        max_iterations: Max optimization iterations per linearization
        max_linesearch_iterations: Max line search iterations
        x0: Initial state [12]
        Sig0: Initial covariance [12, 12]
        xT: Target state [12]
        SigT: Target covariance [12, 12]
        tf: Final time
        sdf_func: Custom SDF function (optional)
        show_iterations: Enable visualization
        obstacles: List of obstacles for visualization
        
    Returns:
        As: Final closed-loop dynamics
        as_: Final closed-loop affine terms
        mean_trj: Final mean trajectory
        cov_trj: Final covariance trajectory
        collision_grad: Final collision gradients
    """
    nt = As.shape[0]
    dt = tf / (nt - 1)
    
    # Ensure positive definite covariances
    Sig0 = ensure_positive_definite(Sig0)
    SigT = ensure_positive_definite(SigT)
    
    # Initialize collision model
    collision_model = Quad3DCollisionSDF(collision_params, quad_params, sdf_func)
    
    # Initialize experiment data
    exp_data = PCS3DExperimentData(
        collision_params=collision_params,
        pcs_params=PCS3DParams(nt, tf, x0, xT, Sig0, SigT, epsilon, eta),
        quad_params=quad_params
    )
    
    # Initialize visualization
    vis_ctx = None
    if show_iterations:
        vis_ctx = Visualization3DContext.create(obstacles)
    
    # Tracking variables
    hA_prev = np.zeros((nt, NX_3D, NX_3D))
    ha_prev = np.zeros((nt, NX_3D))
    current_cost = np.inf
    eta_base = eta
    collision_grad = np.zeros((nt, NX_3D))
    
    # Create linearization function
    def linearize_trj(cov, mean, A):
        return linearize_trajectory_3d(cov, mean, A, quad_params, dt)
    
    # Main loop
    lin_iter = 0
    max_linearization_iters = 50  # Safety limit
    
    while lin_iter < max_linearization_iters:
        print(f"\n{'='*20} Linearization {lin_iter} {'='*20}")
        
        # Linearize along trajectory
        hA, hB, ha, trace_terms = linearize_trj(cov_trj, mean_trj, As)
        lin_iter += 1
        
        # Check for numerical issues in linearization
        if not np.all(np.isfinite(hA)) or not np.all(np.isfinite(hB)) or not np.all(np.isfinite(ha)):
            print("Warning: Non-finite values in linearization, stopping")
            break
        
        # Check convergence
        if _check_convergence(hA, ha, hA_prev, ha_prev):
            print("Converged!")
            break
        
        # Optimization iterations
        for opt_iter in range(max_iterations):
            print(f"\n--- Optimization iteration {opt_iter} ---")
            
            # Visualization
            if vis_ctx is not None:
                vis_ctx.update(x0, xT, mean_trj, opt_iter, quad_params)
            
            # Reset line search
            eta = eta_base
            if opt_iter == 1:
                current_cost = np.inf
            
            # Line search
            converged_ls = False
            for ls_iter in range(1, max_linesearch_iterations + 1):
                print(f"  Line search {ls_iter}")
                eta *= LINE_SEARCH_DECAY
                
                try:
                    # Take step
                    As_new, as_new, mean_new, cov_new, collision_grad, col_cost, ctrl_cost = \
                        _line_search_step(
                            As, as_, Bs, mean_trj,
                            hA, hB, ha, trace_terms,
                            Qt, rt, eta, epsilon,
                            x0, Sig0, xT, SigT, tf,
                            collision_model
                        )
                    
                    new_cost = col_cost + ctrl_cost
                    print(f"    Collision: {col_cost:.4f}, Control: {ctrl_cost:.4f}, Total: {new_cost:.4f}")
                    
                    # Accept if improved
                    if new_cost < current_cost:
                        mean_trj, cov_trj = mean_new, cov_new
                        As, as_ = As_new, as_new
                        current_cost = new_cost
                        converged_ls = True
                        break
                        
                except Exception as e:
                    print(f"    Line search failed: {e}")
                    continue
            
            if not converged_ls:
                print("Line search failed to find improvement")
                hA_prev, ha_prev = hA, ha
                break
            
            # Log iteration
            try:
                Q, r, _, _ = compute_cost_matrices_3d(
                    As, as_, hA, ha, trace_terms, eta, Bs, Qt, rt, mean_trj, collision_model
                )
                A_prox = (eta * As + hA) / (1 + eta)
                a_prox = (eta * as_ + ha) / (1 + eta)
                K, d, _, _ = _safe_linear_covcontrol(
                    A_prox, Bs, a_prox, epsilon, Q, r, x0, Sig0, xT, SigT, tf
                )
                
                exp_data.add_iteration(opt_iter, PCS3DIterationData(
                    As=As.copy(), as_=as_.copy(),
                    hA=hA.copy(), ha=ha.copy(), hB=hB.copy(),
                    Q=Q.copy(), r=r.copy(),
                    mean_trj=mean_trj.copy(), cov_trj=cov_trj.copy(),
                    K=K.copy(), d=d.copy(),
                    cost=current_cost
                ))
            except Exception as e:
                print(f"Warning: Failed to log iteration data: {e}")
        
        if not converged_ls:
            break
    
    # Finalize
    if vis_ctx is not None:
        vis_ctx.finalize()
    
    # Save data
    try:
        os.makedirs(os.path.join(DEBUG_DIR, "PCS3D"), exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        save_path = os.path.join(DEBUG_DIR, "PCS3D", f"data_{timestamp}_{_SCRIPT_NAME}.pkl")
        exp_data.save(save_path)
        print(f"Saved experiment data to {save_path}")
    except Exception as e:
        print(f"Warning: Failed to save experiment data: {e}")
    
    return As, as_, mean_trj, cov_trj, collision_grad


def proximal_cov_steering_3d_with_gains(
    mean_trj: np.ndarray,
    cov_trj: np.ndarray,
    As: np.ndarray,
    Bs: np.ndarray,
    as_: np.ndarray,
    Qt: np.ndarray,
    rt: np.ndarray,
    collision_params: Quad3DCollisionParams,
    quad_params: Quad3DParams,
    epsilon: float,
    eta: float,
    max_iterations: int,
    max_linesearch_iterations: int,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    sdf_func: Optional[Callable] = None,
    xr: Optional[np.ndarray] = None,
    show_iterations: bool = True,
    obstacles: Optional[list] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    PCS with optimal feedback gain recovery.
    
    Returns:
        As, as_, mean_star, cov_star, K, d, Pi_star, lambda_star
    """
    dt = tf / (As.shape[0] - 1)
    
    # Run main optimization
    As, as_, mean_trj, cov_trj, collision_grad = proximal_cov_steering_3d(
        mean_trj, cov_trj, As, Bs, as_, Qt, rt,
        collision_params, quad_params, epsilon, eta,
        max_iterations, max_linesearch_iterations,
        x0, Sig0, xT, SigT, tf, sdf_func, show_iterations, obstacles
    )
    
    # Recover optimal
    mean_star, cov_star = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    hA_star, hB_star, ha_star, trace_terms = linearize_trajectory_3d(
        cov_star, mean_star, As, quad_params, dt
    )
    
    # Cost matrices for recovery
    Q_star = Qt.copy()
    r_star = trace_terms / 2.0 + collision_grad
    
    if xr is not None:
        for i in range(Qt.shape[0]):
            r_star[i] = r_star[i] - Qt[i] @ xr[i]
    
    K, d, Pi_star, lambda_star = _safe_linear_covcontrol(
        hA_star, hB_star, ha_star, epsilon, Q_star, r_star, x0, Sig0, xT, SigT, tf
    )
    
    return As, as_, mean_star, cov_star, K, d, Pi_star, lambda_star


def proximal_cov_steering_3d_from_scratch(
    Qt: np.ndarray,
    rt: np.ndarray,
    collision_params: Quad3DCollisionParams,
    quad_params: Quad3DParams,
    epsilon: float,
    eta: float,
    max_iterations: int,
    max_linesearch_iterations: int,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    nt: int,
    sdf_func: Optional[Callable] = None,
    show_iterations: bool = True,
    obstacles: Optional[list] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Initialize and run PCS from boundary conditions only.
    
    Returns:
        As, as_, K, d, Pi_star, lambda_star
    """
    dt = tf / (nt - 1)
    
    print("Initializing trajectory...")
    
    # Use dynamics-aware initialization
    mean_init, cov_init, As, as_ = initialize_trajectory_interpolation(
        x0, xT, nt, tf, quad_params
    )
    
    # Get B matrices from linearization
    Bs = np.zeros((nt, NX_3D, NU_3D), dtype=np.float64)
    for i in range(nt):
        _, Bs[i], _ = linearize_point_3d(mean_init[i], quad_params, dt)
    
    print(f"Initial trajectory shape: {mean_init.shape}")
    print(f"Initial position: {mean_init[0, 0:3]}")
    print(f"Final position: {mean_init[-1, 0:3]}")
    
    # Format obstacles for visualization
    vis_obstacles = obstacles
    
    # =========================================================================
    # Run optimization with gain tracking
    # =========================================================================
    As, as_, mean_trj, cov_trj, collision_grad, last_K, last_d = \
        _proximal_cov_steering_3d_with_gain_tracking(
            mean_init, cov_init, As, Bs, as_, Qt, rt,
            collision_params, quad_params, epsilon, eta,
            max_iterations, max_linesearch_iterations,
            x0, Sig0, xT, SigT, tf, sdf_func, show_iterations, vis_obstacles
        )
    
    # Try to recover optimal gains, fall back to last valid gains if it fails
    try:
        mean_star, cov_star = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
        hA_star, hB_star, ha_star, trace_terms = linearize_trajectory_3d(
            cov_star, mean_star, As, quad_params, dt
        )
        
        Q_star = Qt.copy()
        r_star = trace_terms / 2.0 + collision_grad
        
        K, d, Pi_star, lambda_star = _safe_linear_covcontrol(
            hA_star, hB_star, ha_star, epsilon, Q_star, r_star, x0, Sig0, xT, SigT, tf
        )
        print("Successfully recovered optimal gains.")
        
    except Exception as e:
        print(f"Warning: Could not recover optimal gains ({e}), using last valid gains.")
        K = last_K
        d = last_d
        Pi_star = np.zeros((nt, NX_3D, NX_3D))
        lambda_star = np.zeros((nt, NX_3D))
    
    return As, as_, K, d, Pi_star, lambda_star


def _proximal_cov_steering_3d_with_gain_tracking(
    mean_trj: np.ndarray,
    cov_trj: np.ndarray,
    As: np.ndarray,
    Bs: np.ndarray,
    as_: np.ndarray,
    Qt: np.ndarray,
    rt: np.ndarray,
    collision_params: Quad3DCollisionParams,
    quad_params: Quad3DParams,
    epsilon: float,
    eta: float,
    max_iterations: int,
    max_linesearch_iterations: int,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    sdf_func: Optional[Callable] = None,
    show_iterations: bool = True,
    obstacles: Optional[list] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Internal PCS function that also tracks and returns the last valid gains.
    
    Returns:
        As, as_, mean_trj, cov_trj, collision_grad, last_K, last_d
    """
    nt = As.shape[0]
    dt = tf / (nt - 1)
    
    # Ensure positive definite covariances
    Sig0 = ensure_positive_definite(Sig0)
    SigT = ensure_positive_definite(SigT)
    
    # Initialize collision model
    collision_model = Quad3DCollisionSDF(collision_params, quad_params, sdf_func)
    
    # Initialize experiment data
    exp_data = PCS3DExperimentData(
        collision_params=collision_params,
        pcs_params=PCS3DParams(nt, tf, x0, xT, Sig0, SigT, epsilon, eta),
        quad_params=quad_params
    )
    
    # Initialize visualization
    vis_ctx = None
    if show_iterations:
        vis_ctx = Visualization3DContext.create(obstacles)
    
    # Tracking variables
    hA_prev = np.zeros((nt, NX_3D, NX_3D))
    ha_prev = np.zeros((nt, NX_3D))
    current_cost = np.inf
    eta_base = eta
    collision_grad = np.zeros((nt, NX_3D))
    
    # Track last valid gains
    last_K = np.zeros((nt, NU_3D, NX_3D))
    last_d = np.zeros((nt, NU_3D))
    
    # Create linearization function
    def linearize_trj(cov, mean, A):
        return linearize_trajectory_3d(cov, mean, A, quad_params, dt)
    
    # Main loop
    lin_iter = 0
    max_linearization_iters = 50
    
    while lin_iter < max_linearization_iters:
        print(f"\n{'='*20} Linearization {lin_iter} {'='*20}")
        
        # Linearize along trajectory
        hA, hB, ha, trace_terms = linearize_trj(cov_trj, mean_trj, As)
        lin_iter += 1
        
        # Check for numerical issues
        if not np.all(np.isfinite(hA)) or not np.all(np.isfinite(hB)) or not np.all(np.isfinite(ha)):
            print("Warning: Non-finite values in linearization, stopping")
            break
        
        # Check convergence
        if _check_convergence(hA, ha, hA_prev, ha_prev):
            print("Converged!")
            break
        
        # Optimization iterations
        for opt_iter in range(max_iterations):
            print(f"\n--- Optimization iteration {opt_iter} ---")
            
            if vis_ctx is not None:
                vis_ctx.update(x0, xT, mean_trj, opt_iter, quad_params)
            
            eta = eta_base
            if opt_iter == 1:
                current_cost = np.inf
            
            # Line search
            converged_ls = False
            for ls_iter in range(1, max_linesearch_iterations + 1):
                print(f"  Line search {ls_iter}")
                eta *= LINE_SEARCH_DECAY
                
                try:
                    As_new, as_new, mean_new, cov_new, collision_grad_new, col_cost, ctrl_cost, K_new, d_new = \
                        _line_search_step_with_gains(
                            As, as_, Bs, mean_trj,
                            hA, hB, ha, trace_terms,
                            Qt, rt, eta, epsilon,
                            x0, Sig0, xT, SigT, tf,
                            collision_model
                        )
                    
                    new_cost = col_cost + ctrl_cost
                    print(f"    Collision: {col_cost:.4f}, Control: {ctrl_cost:.4f}, Total: {new_cost:.4f}")
                    
                    if new_cost < current_cost:
                        mean_trj, cov_trj = mean_new, cov_new
                        As, as_ = As_new, as_new
                        collision_grad = collision_grad_new
                        current_cost = new_cost
                        
                        # Save the valid gains
                        last_K = K_new
                        last_d = d_new
                        
                        converged_ls = True
                        break
                        
                except Exception as e:
                    print(f"    Line search failed: {e}")
                    continue
            
            if not converged_ls:
                print("Line search failed to find improvement")
                hA_prev, ha_prev = hA, ha
                break
        
        if not converged_ls:
            break
    
    # Finalize
    if vis_ctx is not None:
        vis_ctx.finalize()
    
    # Save data
    try:
        os.makedirs(os.path.join(DEBUG_DIR, "PCS3D"), exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        save_path = os.path.join(DEBUG_DIR, "PCS3D", f"data_{timestamp}_{_SCRIPT_NAME}.pkl")
        exp_data.save(save_path)
        print(f"Saved experiment data to {save_path}")
    except Exception as e:
        print(f"Warning: Failed to save experiment data: {e}")
    
    return As, as_, mean_trj, cov_trj, collision_grad, last_K, last_d


def _line_search_step_with_gains(
    As: np.ndarray,
    as_: np.ndarray,
    Bs: np.ndarray,  # This will be ignored, we use hB instead
    mean_trj: np.ndarray,
    hA: np.ndarray,
    hB: np.ndarray,  # Use this consistently
    ha: np.ndarray,
    trace_terms: np.ndarray,
    Qt: np.ndarray,
    rt: np.ndarray,
    eta: float,
    epsilon: float,
    x0: np.ndarray,
    Sig0: np.ndarray,
    xT: np.ndarray,
    SigT: np.ndarray,
    tf: float,
    collision_model: Quad3DCollisionSDF
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float, np.ndarray, np.ndarray]:
    """
    Line search step with consistent B matrix usage.
    """
    nt, nx, _ = As.shape
    
    # Compute cost matrices
    Q, r, collision_cost, collision_grad = compute_cost_matrices_3d(
        As, as_, hA, ha, trace_terms, eta, hB, Qt, rt, mean_trj, collision_model
    )
    
    # Proximal dynamics
    A_prox = (eta * As + hA) / (1 + eta)
    a_prox = (eta * as_ + ha) / (1 + eta)
    
    # KEY FIX: Use hB (not Bs) for covariance steering
    K, d, _, _ = _safe_linear_covcontrol(
        A_prox, hB, a_prox, epsilon, Q, r, x0, Sig0, xT, SigT, tf  # Changed Bs to hB
    )
    
    # Update closed-loop dynamics with SAME hB
    As_new = np.zeros_like(As)
    as_new = np.zeros_like(as_)
    
    for i in range(nt):
        As_new[i] = A_prox[i] + hB[i] @ K[i]
        as_new[i] = a_prox[i] + hB[i] @ d[i]
    
    # Propagate with hB (not Bs)
    mean_new, cov_new = mean_cov_cl(As_new, hB, as_new, epsilon, x0, Sig0, tf)  # Changed Bs to hB
    
    if not np.all(np.isfinite(mean_new)):
        raise ValueError("Non-finite values in propagated mean trajectory")
    
    # Control cost
    control_signal = compute_control_signal(x0, K, d)
    control_cost = np.linalg.norm(control_signal) / 2.0
    
    return As_new, as_new, mean_new, cov_new, collision_grad, collision_cost, control_cost, K, d



# ============================================================================
# Convenience Functions
# ============================================================================
def construct_cost_matrices_3d(
    x0: np.ndarray,
    xT: np.ndarray,
    nt: int,
    position_weight: float = 0.0,
    velocity_weight: float = 0.0
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Construct running cost matrices for 3D quadrotor.
    
    Args:
        x0: Initial state
        xT: Target state
        nt: Number of timesteps
        position_weight: Weight on position tracking
        velocity_weight: Weight on velocity tracking
        
    Returns:
        Qt: Cost matrices [nt, 12, 12]
        rt: Cost linear terms [nt, 12]
    """
    Qt = np.zeros((nt, NX_3D, NX_3D), dtype=np.float64)
    rt = np.zeros((nt, NX_3D), dtype=np.float64)
    
    # Optionally add tracking costs
    for i in range(nt):
        Qt[i, 0:3, 0:3] = position_weight * np.eye(3)
        Qt[i, 6:9, 6:9] = velocity_weight * np.eye(3)
        
        # Add small regularization
        Qt[i] += COVARIANCE_REGULARIZATION * np.eye(NX_3D)
    
    return Qt, rt


initialize_straight_line_trajectory = initialize_trajectory_interpolation

