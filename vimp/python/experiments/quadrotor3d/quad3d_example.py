"""
Example: Collision-Avoiding Trajectory Planning for 3D Quadrotor

This script demonstrates the Proximal Covariance Steering (PCS) algorithm
for a 3D quadrotor navigating through obstacles.

Author: Adapted from planar quadrotor experiments
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Dict, Any
import os

# Import the PCS module
from experiments.quadrotor3d.pcs_quad3d import (
    proximal_cov_steering_3d,
    proximal_cov_steering_3d_with_gains,
    proximal_cov_steering_3d_from_scratch,
    construct_cost_matrices_3d,
    initialize_trajectory_interpolation,
    Quad3DCollisionParams,
    PCS3DParams,
    NX_3D,
    NU_3D,
    linearize_point_3d, 
)
from dynamics.quad3d import Quad3DParams, simulate_quad3d, hover_equilibrium
from tools.propagations import mean_cov_cl

# ============================================================================
# SDF Functions for Different Obstacle Configurations
# ============================================================================

def create_sphere_obstacle_sdf(centers: List[np.ndarray], radii: List[float]):
    """
    Create SDF function for multiple spherical obstacles.
    
    Args:
        centers: List of obstacle center positions
        radii: List of obstacle radii
        
    Returns:
        sdf_func: Function that returns (distance, gradient) for a query point
    """
    def sdf_func(position: np.ndarray) -> Tuple[float, np.ndarray]:
        min_dist = np.inf
        min_grad = np.zeros(3)
        
        for center, radius in zip(centers, radii):
            diff = position - center
            dist = np.linalg.norm(diff)
            
            if dist < 1e-6:
                # Inside obstacle center
                signed_dist = -radius
                grad = np.zeros(3)
            else:
                signed_dist = dist - radius
                grad = diff / dist
            
            if signed_dist < min_dist:
                min_dist = signed_dist
                min_grad = grad
        
        return min_dist, min_grad
    
    return sdf_func


def create_cylinder_obstacle_sdf(
    center: np.ndarray, 
    radius: float, 
    height: float,
    axis: str = 'z'
):
    """
    Create SDF for a vertical cylinder obstacle.
    
    Args:
        center: Center of cylinder base
        radius: Cylinder radius
        height: Cylinder height
        axis: Axis of cylinder ('x', 'y', or 'z')
    """
    def sdf_func(position: np.ndarray) -> Tuple[float, np.ndarray]:
        if axis == 'z':
            # Project to xy plane
            diff_xy = position[0:2] - center[0:2]
            dist_xy = np.linalg.norm(diff_xy)
            
            # Radial distance
            radial_dist = dist_xy - radius
            
            # Vertical distance
            z_rel = position[2] - center[2]
            if z_rel < 0:
                vertical_dist = -z_rel
            elif z_rel > height:
                vertical_dist = z_rel - height
            else:
                vertical_dist = -min(z_rel, height - z_rel)
            
            # Combined SDF
            if radial_dist > 0 and vertical_dist > 0:
                # Outside corner
                signed_dist = np.sqrt(radial_dist**2 + vertical_dist**2)
                grad = np.zeros(3)
                if dist_xy > 1e-6:
                    grad[0:2] = diff_xy / dist_xy * radial_dist / signed_dist
                grad[2] = np.sign(z_rel - height/2) * vertical_dist / signed_dist
            elif radial_dist > vertical_dist:
                signed_dist = radial_dist
                grad = np.zeros(3)
                if dist_xy > 1e-6:
                    grad[0:2] = diff_xy / dist_xy
            else:
                signed_dist = vertical_dist
                grad = np.array([0, 0, np.sign(z_rel - height/2)])
            
            return signed_dist, grad
        else:
            raise NotImplementedError(f"Axis {axis} not implemented")
    
    return sdf_func


def create_combined_sdf(sdf_funcs: List):
    """Combine multiple SDF functions (union of obstacles)."""
    def combined_sdf(position: np.ndarray) -> Tuple[float, np.ndarray]:
        min_dist = np.inf
        min_grad = np.zeros(3)
        
        for sdf_func in sdf_funcs:
            dist, grad = sdf_func(position)
            if dist < min_dist:
                min_dist = dist
                min_grad = grad
        
        return min_dist, min_grad
    
    return combined_sdf


# ============================================================================
# Experiment Configurations
# ============================================================================

def get_experiment_config(exp_index: int) -> Dict[str, Any]:
    """
    Get experiment configuration by index.
    
    Args:
        exp_index: Experiment index (1-4)
        
    Returns:
        Configuration dictionary
    """
    configs = {
        1: {
            # Simple point-to-point with single obstacle
            'name': 'Single Sphere Avoidance',
            'x0': np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            'xT': np.array([10.0, 10.0, 5.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            'obstacles': [
                {'center': np.array([5.0, 5.0, 3.0]), 'radius': 1.0},
            ],
            'eps_obs': 0.5,
            'slope': 100.0,
            'sig_obs': 20.0,
            'robot_radius': 0.3,
            'tf': 2.5,
        },
    }
    
    return configs[exp_index]


# ============================================================================
# Visualization
# ============================================================================

def plot_final_trajectory_3d(
    mean_trj: np.ndarray,
    x0: np.ndarray,
    xT: np.ndarray,
    obstacles: List[Dict],
    quad_params: Quad3DParams,
    title: str = "3D Quadrotor Trajectory",
    save_path: str = None
):
    """
    Create publication-quality 3D trajectory plot.
    """
    from mpl_toolkits.mplot3d import Axes3D
    from dynamics.quad3d import rotation_matrix_zyx
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot trajectory
    ax.plot(mean_trj[:, 0], mean_trj[:, 1], mean_trj[:, 2],
            'b-', linewidth=2.5, label='Optimal Trajectory')
    
    # Plot start and goal
    ax.scatter(*x0[0:3], c='green', s=200, marker='o', 
               label='Start', edgecolors='darkgreen', linewidths=2)
    ax.scatter(*xT[0:3], c='red', s=200, marker='*', 
               label='Goal', edgecolors='darkred', linewidths=2)
    
    # Plot obstacles as wireframe spheres
    for obs in obstacles:
        center = obs['center']
        radius = obs['radius']
        
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 20)
        
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        ax.plot_surface(x, y, z, color='red', alpha=0.3)
        ax.plot_wireframe(x, y, z, color='darkred', alpha=0.5, linewidth=0.5)
    
    # Plot quadrotor frames along trajectory
    n_frames = 25
    frame_indices = np.linspace(0, len(mean_trj) - 1, n_frames, dtype=int)
    
    for idx in frame_indices:
        state = mean_trj[idx]
        position = state[0:3]
        phi, theta, psi = state[3], state[4], state[5]
        R = rotation_matrix_zyx(phi, theta, psi)
        
        L = quad_params.arm_length
        
        # Rotor positions
        rotor_body = [
            np.array([L, 0, 0]),
            np.array([-L, 0, 0]),
            np.array([0, L, 0]),
            np.array([0, -L, 0]),
        ]
        
        colors = ['red', 'red', 'blue', 'blue']  # Front rotors red, rear blue
        
        for rotor, color in zip(rotor_body, colors):
            rotor_world = position + R @ rotor
            ax.plot([position[0], rotor_world[0]],
                   [position[1], rotor_world[1]],
                   [position[2], rotor_world[2]],
                   'k-', linewidth=2, alpha=0.7)
            ax.scatter(*rotor_world, c=color, s=50, alpha=0.7)
    
    # Labels and formatting
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_zlabel('Z [m]', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.legend(loc='upper left', fontsize=10)
    
    # Equal aspect ratio
    max_range = np.array([
        mean_trj[:, 0].max() - mean_trj[:, 0].min(),
        mean_trj[:, 1].max() - mean_trj[:, 1].min(),
        mean_trj[:, 2].max() - mean_trj[:, 2].min()
    ]).max() / 2.0
    
    mid_x = (mean_trj[:, 0].max() + mean_trj[:, 0].min()) * 0.5
    mid_y = (mean_trj[:, 1].max() + mean_trj[:, 1].min()) * 0.5
    mid_z = (mean_trj[:, 2].max() + mean_trj[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved figure to {save_path}")
    
    plt.show()


def plot_state_trajectories(
    mean_trj: np.ndarray,
    tf: float,
    title: str = "State Trajectories",
    save_path: str = None
):
    """Plot state components over time."""
    nt = mean_trj.shape[0]
    t = np.linspace(0, tf, nt)
    
    fig, axes = plt.subplots(4, 3, figsize=(15, 12))
    
    state_names = [
        'x [m]', 'y [m]', 'z [m]',
        'φ [rad]', 'θ [rad]', 'ψ [rad]',
        'vx [m/s]', 'vy [m/s]', 'vz [m/s]',
        'p [rad/s]', 'q [rad/s]', 'r [rad/s]'
    ]
    
    for i, (ax, name) in enumerate(zip(axes.flat, state_names)):
        ax.plot(t, mean_trj[:, i], 'b-', linewidth=1.5)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
        ax.set_xlim([0, tf])
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.show()


# ============================================================================
# Main Experiment Runner
# ============================================================================

def run_single_experiment(
    exp_index: int,
    show_iterations: bool = True,
    save_results: bool = True
) -> Dict[str, Any]:
    """
    Run a single PCS experiment.
    """
    print(f"\n{'='*60}")
    print(f"Running Experiment {exp_index}")
    print('='*60)
    
    # Get configuration
    config = get_experiment_config(exp_index)
    print(f"Configuration: {config['name']}")
    
    # Extract parameters
    x0 = config['x0'].copy()
    xT = config['xT'].copy()
    obstacles = config['obstacles']
    tf = config['tf']
    
    # Quadrotor parameters
    quad_params = Quad3DParams(
        mass=1.0,
        Jx=0.01,
        Jy=0.01,
        Jz=0.02,
        arm_length=0.25,
        g=9.81,
        body_radius=0.1,
        rotor_radius=0.05,
        use_rotor_spheres=True,
    )
    
    # Collision parameters
    collision_params = Quad3DCollisionParams(
        eps_obs=config['eps_obs'],
        slope=config['slope'],
        sig_obs=config['sig_obs'],
        radius=config['robot_radius'],
        map_name=config['name'],
    )
    
    # Create SDF function
    obstacle_centers = [obs['center'] for obs in obstacles]
    obstacle_radii = [obs['radius'] for obs in obstacles]
    sdf_func = create_sphere_obstacle_sdf(obstacle_centers, obstacle_radii)
    
    # Algorithm parameters - ADJUSTED FOR STABILITY
    nt = 100 
    dt = tf / (nt - 1)
    epsilon = 1.0  # Reduced regularization
    eta = 1.0  # Increased proximal parameter
    max_iterations = 30
    max_linesearch = 100  # Increased line search iterations
    
    # Boundary covariances - INCREASED FOR STABILITY
    coeff_Sig0 = 0.5
    coeff_SigT = 0.1
    Sig0 = np.eye(NX_3D) * coeff_Sig0
    SigT = np.eye(NX_3D) * coeff_SigT
    
    # Cost matrices with small regularization
    Qt, rt = construct_cost_matrices_3d(x0, xT, nt, position_weight=10.0, velocity_weight=0.01)
    
    # Format obstacles for visualization
    vis_obstacles = [
        {'center': obs['center'], 'radius': obs['radius']}
        for obs in obstacles
    ]
    
    # Use the from_scratch interface with proper initialization
    print("\nStarting PCS optimization...")
    
    try:
        As, as_, K, d, Pi, lbd = proximal_cov_steering_3d_from_scratch(
            Qt=Qt,
            rt=rt,
            collision_params=collision_params,
            quad_params=quad_params,
            epsilon=epsilon,
            eta=eta,
            max_iterations=max_iterations,
            max_linesearch_iterations=max_linesearch,
            x0=x0,
            Sig0=Sig0,
            xT=xT,
            SigT=SigT,
            tf=tf,
            nt=nt,
            sdf_func=sdf_func,
            show_iterations=show_iterations,
            obstacles=vis_obstacles,
        )
        
        print("\nOptimization complete!")
        
        # Get final trajectory
        Bs = np.zeros((nt, NX_3D, NU_3D))
        for i in range(nt):
            _, Bs[i], _ = linearize_point_3d(x0, quad_params, dt)
        
        mean_final, cov_final = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
        
        # Check boundary conditions
        print(f"\nBoundary condition errors:")
        print(f"  Initial position error: {np.linalg.norm(mean_final[0, 0:3] - x0[0:3]):.6f}")
        print(f"  Final position error: {np.linalg.norm(mean_final[-1, 0:3] - xT[0:3]):.6f}")
        
        # Plot results
        plot_final_trajectory_3d(
            mean_final, x0, xT, obstacles, quad_params,
            title=f"Experiment {exp_index}: {config['name']}",
            save_path=f"exp{exp_index}_trajectory_3d.png" if save_results else None
        )
        
        # Compile results
        results = {
            'exp_index': exp_index,
            'config': config,
            'mean_trajectory': mean_final,
            'covariance_trajectory': cov_final,
            'As': As,
            'as_': as_,
            'K': K,
            'd': d,
            'quad_params': quad_params,
            'collision_params': collision_params,
        }
        
        return results
        
    except Exception as e:
        print(f"Experiment failed: {e}")
        import traceback
        traceback.print_exc()
        return {'error': str(e)}
    
    
def run_all_experiments(show_iterations: bool = False):
    """Run all experiment configurations."""
    all_results = {}
    
    for exp_index in range(1, 5):
        results = run_single_experiment(
            exp_index,
            show_iterations=show_iterations,
            save_results=True
        )
        all_results[exp_index] = results
    
    # Save combined results
    import pickle
    with open('all_experiments_results.pkl', 'wb') as f:
        pickle.dump(all_results, f)
    
    print("\n" + "="*60)
    print("All experiments completed!")
    print("="*60)
    
    return all_results


# ============================================================================
# Alternative: Using the from_scratch interface
# ============================================================================

def run_experiment_from_scratch(exp_index: int = 1):
    """
    Example using the from_scratch interface that handles initialization.
    """
    print(f"\n{'='*60}")
    print(f"Running Experiment {exp_index} (from scratch interface)")
    print('='*60)
    
    config = get_experiment_config(exp_index)
    
    x0 = config['x0'].copy()
    xT = config['xT'].copy()
    tf = config['tf']
    
    # Set initial velocities
    x0[6:9] = (xT[0:3] - x0[0:3]) / tf
    
    quad_params = Quad3DParams()
    
    collision_params = Quad3DCollisionParams(
        eps_obs=config['eps_obs'],
        slope=config['slope'],
        sig_obs=config['sig_obs'],
        radius=config['robot_radius'],
    )
    
    # Create SDF
    obstacle_centers = [obs['center'] for obs in config['obstacles']]
    obstacle_radii = [obs['radius'] for obs in config['obstacles']]
    sdf_func = create_sphere_obstacle_sdf(obstacle_centers, obstacle_radii)
    
    # Parameters
    nt = 2000
    epsilon = 1.0
    eta = 0.3
    
    Sig0 = np.eye(NX_3D) * 0.01
    SigT = np.eye(NX_3D) * 0.01
    Qt, rt = construct_cost_matrices_3d(x0, xT, nt)
    
    vis_obstacles = [
        {'center': obs['center'], 'radius': obs['radius']}
        for obs in config['obstacles']
    ]
    
    # Run from scratch - handles all initialization internally
    As, as_, K, d, Pi, lbd = proximal_cov_steering_3d_from_scratch(
        Qt=Qt,
        rt=rt,
        collision_params=collision_params,
        quad_params=quad_params,
        epsilon=epsilon,
        eta=eta,
        max_iterations=15,
        max_linesearch_iterations=5,
        x0=x0,
        Sig0=Sig0,
        xT=xT,
        SigT=SigT,
        tf=tf,
        nt=nt,
        sdf_func=sdf_func,
        show_iterations=True,
        obstacles=vis_obstacles,
    )
    
    print("Optimization complete!")
    print(f"Feedback gain K shape: {K.shape}")
    print(f"Feedforward d shape: {d.shape}")
    
    return As, as_, K, d


# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='3D Quadrotor PCS Experiments')
    parser.add_argument('--exp', type=int, default=1, choices=[1, 2, 3, 4],
                        help='Experiment index (1-4)')
    parser.add_argument('--all', action='store_true',
                        help='Run all experiments')
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable real-time visualization')
    parser.add_argument('--from-scratch', action='store_true',
                        help='Use from_scratch interface')
    
    args = parser.parse_args()
    
    if args.all:
        run_all_experiments(show_iterations=not args.no_viz)
    elif args.from_scratch:
        run_experiment_from_scratch(args.exp)
    else:
        run_single_experiment(
            args.exp,
            show_iterations=not args.no_viz,
            save_results=True
        )