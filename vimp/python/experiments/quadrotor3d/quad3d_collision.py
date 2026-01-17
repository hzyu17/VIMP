"""
3D Collision Checking for Quadrotor using Signed Distance Fields (SDF)

Represents quadrotor as multiple spheres:
- 1 body sphere at center
- 4 rotor spheres at arm endpoints (optional)

Computes hinge loss and Jacobians for obstacle avoidance.
"""

import numpy as np
from typing import Tuple, List, Optional, Callable
from dataclasses import dataclass

from vimp.python.dynamics.quad3d import Quad3DParams, rotation_matrix_zyx, rotation_matrix_derivatives


@dataclass
class CollisionParams:
    """Parameters for collision checking."""
    eps_sdf: float = 0.1      # Safety margin
    sig_obs: float = 100.0    # Obstacle cost weight
    slope: float = 1.0        # Hinge loss slope (for smooth approximation)


class SDF3D:
    """
    Base class for 3D Signed Distance Field.
    
    Positive values = outside obstacle (free space)
    Negative values = inside obstacle
    """
    
    def query(self, point: np.ndarray) -> float:
        """Query SDF value at point."""
        raise NotImplementedError
    
    def gradient(self, point: np.ndarray) -> np.ndarray:
        """Query SDF gradient at point."""
        raise NotImplementedError
    
    def query_batch(self, points: np.ndarray) -> np.ndarray:
        """Query SDF values at multiple points [N x 3] -> [N]."""
        return np.array([self.query(p) for p in points])
    
    def gradient_batch(self, points: np.ndarray) -> np.ndarray:
        """Query SDF gradients at multiple points [N x 3] -> [N x 3]."""
        return np.array([self.gradient(p) for p in points])


class SphereSDF(SDF3D):
    """SDF for a single sphere obstacle."""
    
    def __init__(self, center: np.ndarray, radius: float):
        self.center = np.asarray(center)
        self.radius = radius
    
    def query(self, point: np.ndarray) -> float:
        return np.linalg.norm(point - self.center) - self.radius
    
    def gradient(self, point: np.ndarray) -> np.ndarray:
        diff = point - self.center
        dist = np.linalg.norm(diff)
        if dist < 1e-8:
            return np.array([1.0, 0.0, 0.0])
        return diff / dist


class BoxSDF(SDF3D):
    """SDF for an axis-aligned box obstacle."""
    
    def __init__(self, center: np.ndarray, half_extents: np.ndarray):
        self.center = np.asarray(center)
        self.half_extents = np.asarray(half_extents)
    
    def query(self, point: np.ndarray) -> float:
        # Transform to box local frame
        q = np.abs(point - self.center) - self.half_extents
        
        # Distance computation
        outside = np.maximum(q, 0)
        inside = min(max(q[0], max(q[1], q[2])), 0)
        
        return np.linalg.norm(outside) + inside
    
    def gradient(self, point: np.ndarray) -> np.ndarray:
        # Numerical gradient
        eps = 1e-5
        grad = np.zeros(3)
        for i in range(3):
            p_plus = point.copy()
            p_minus = point.copy()
            p_plus[i] += eps
            p_minus[i] -= eps
            grad[i] = (self.query(p_plus) - self.query(p_minus)) / (2 * eps)
        return grad


class CompositeSDF(SDF3D):
    """SDF combining multiple obstacles (union = min of SDFs)."""
    
    def __init__(self, sdfs: List[SDF3D]):
        self.sdfs = sdfs
    
    def query(self, point: np.ndarray) -> float:
        return min(sdf.query(point) for sdf in self.sdfs)
    
    def gradient(self, point: np.ndarray) -> np.ndarray:
        # Gradient of the closest obstacle
        min_dist = float('inf')
        min_grad = np.zeros(3)
        
        for sdf in self.sdfs:
            d = sdf.query(point)
            if d < min_dist:
                min_dist = d
                min_grad = sdf.gradient(point)
        
        return min_grad


class GridSDF(SDF3D):
    """SDF from a 3D voxel grid with trilinear interpolation."""
    
    def __init__(self, grid: np.ndarray, origin: np.ndarray, 
                 resolution: float):
        """
        Args:
            grid: 3D array of SDF values [nx, ny, nz]
            origin: World position of grid[0,0,0]
            resolution: Grid cell size
        """
        self.grid = grid
        self.origin = np.asarray(origin)
        self.resolution = resolution
        self.shape = np.array(grid.shape)
    
    def _world_to_grid(self, point: np.ndarray) -> np.ndarray:
        """Convert world coordinates to grid coordinates."""
        return (point - self.origin) / self.resolution
    
    def query(self, point: np.ndarray) -> float:
        """Trilinear interpolation of SDF value."""
        gc = self._world_to_grid(point)
        
        # Clamp to grid bounds
        gc = np.clip(gc, 0, self.shape - 1 - 1e-6)
        
        # Integer and fractional parts
        i0 = gc.astype(int)
        i1 = np.minimum(i0 + 1, self.shape - 1)
        f = gc - i0
        
        # Trilinear interpolation
        c000 = self.grid[i0[0], i0[1], i0[2]]
        c001 = self.grid[i0[0], i0[1], i1[2]]
        c010 = self.grid[i0[0], i1[1], i0[2]]
        c011 = self.grid[i0[0], i1[1], i1[2]]
        c100 = self.grid[i1[0], i0[1], i0[2]]
        c101 = self.grid[i1[0], i0[1], i1[2]]
        c110 = self.grid[i1[0], i1[1], i0[2]]
        c111 = self.grid[i1[0], i1[1], i1[2]]
        
        c00 = c000 * (1-f[0]) + c100 * f[0]
        c01 = c001 * (1-f[0]) + c101 * f[0]
        c10 = c010 * (1-f[0]) + c110 * f[0]
        c11 = c011 * (1-f[0]) + c111 * f[0]
        
        c0 = c00 * (1-f[1]) + c10 * f[1]
        c1 = c01 * (1-f[1]) + c11 * f[1]
        
        return c0 * (1-f[2]) + c1 * f[2]
    
    def gradient(self, point: np.ndarray) -> np.ndarray:
        """Gradient via central differences."""
        eps = self.resolution * 0.5
        grad = np.zeros(3)
        
        for i in range(3):
            p_plus = point.copy()
            p_minus = point.copy()
            p_plus[i] += eps
            p_minus[i] -= eps
            grad[i] = (self.query(p_plus) - self.query(p_minus)) / (2 * eps)
        
        # Normalize if non-zero
        norm = np.linalg.norm(grad)
        if norm > 1e-8:
            grad = grad / norm
        
        return grad


class Quad3DCollision:
    """
    Collision checking for 3D quadrotor using SDF.
    
    Represents quadrotor as multiple spheres and computes
    hinge loss + Jacobians for optimization.
    """
    
    def __init__(self, sdf: SDF3D, quad_params: Quad3DParams, 
                 collision_params: CollisionParams):
        self.sdf = sdf
        self.quad_params = quad_params
        self.collision_params = collision_params
        
        # Number of collision spheres
        self.n_spheres = 5 if quad_params.use_rotor_spheres else 1
        
        # Sphere radii
        self.radii = np.zeros(self.n_spheres)
        self.radii[0] = quad_params.body_radius
        if quad_params.use_rotor_spheres:
            self.radii[1:5] = quad_params.rotor_radius
    
    def compute_sphere_positions(self, z: np.ndarray) -> np.ndarray:
        """
        Compute world positions of all collision spheres.
        
        Args:
            z: Quadrotor state [12]
        
        Returns:
            positions: Sphere centers [n_spheres x 3]
        """
        pos = z[0:3]
        phi, theta, psi = z[3], z[4], z[5]
        
        R = rotation_matrix_zyx(phi, theta, psi)
        
        positions = np.zeros((self.n_spheres, 3))
        positions[0] = pos  # Body center
        
        if self.quad_params.use_rotor_spheres:
            L = self.quad_params.arm_length
            # Rotor positions in body frame (X configuration)
            rotor_body = np.array([
                [L, 0, 0],
                [0, L, 0],
                [-L, 0, 0],
                [0, -L, 0]
            ])
            
            for i in range(4):
                positions[i+1] = pos + R @ rotor_body[i]
        
        return positions
    
    def compute_sphere_jacobians(self, z: np.ndarray) -> np.ndarray:
        """
        Compute Jacobians of sphere positions w.r.t. state.
        
        Args:
            z: Quadrotor state [12]
        
        Returns:
            jacobians: [n_spheres x 3 x 12]
        """
        phi, theta, psi = z[3], z[4], z[5]
        
        jacobians = np.zeros((self.n_spheres, 3, 12))
        
        # Body center: only depends on position
        jacobians[0, 0:3, 0:3] = np.eye(3)
        
        if self.quad_params.use_rotor_spheres:
            L = self.quad_params.arm_length
            dR_dphi, dR_dtheta, dR_dpsi = rotation_matrix_derivatives(phi, theta, psi)
            
            rotor_body = np.array([
                [L, 0, 0],
                [0, L, 0],
                [-L, 0, 0],
                [0, -L, 0]
            ])
            
            for i in range(4):
                # Position contribution
                jacobians[i+1, 0:3, 0:3] = np.eye(3)
                
                # Attitude contribution: d(R @ r)/d(euler)
                jacobians[i+1, :, 3] = dR_dphi @ rotor_body[i]
                jacobians[i+1, :, 4] = dR_dtheta @ rotor_body[i]
                jacobians[i+1, :, 5] = dR_dpsi @ rotor_body[i]
        
        return jacobians
    
    def hinge_loss(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute hinge loss and Jacobian for all collision spheres.
        
        Hinge loss: h_i = max(0, eps + r_i - sdf(p_i))
        
        Args:
            z: Quadrotor state [12]
        
        Returns:
            hinge: Hinge loss vector [n_spheres]
            J_hinge: Jacobian [n_spheres x 12]
        """
        eps = self.collision_params.eps_sdf
        
        # Get sphere positions and Jacobians
        positions = self.compute_sphere_positions(z)
        jacobians = self.compute_sphere_jacobians(z)
        
        hinge = np.zeros(self.n_spheres)
        J_hinge = np.zeros((self.n_spheres, 12))
        
        for i in range(self.n_spheres):
            pos = positions[i]
            
            # Query SDF
            sdf_val = self.sdf.query(pos)
            sdf_grad = self.sdf.gradient(pos)
            
            # Signed distance with safety margin
            d = sdf_val - self.radii[i] - eps
            
            # Hinge loss: max(0, -d)
            if d < 0:
                hinge[i] = -d
                # Chain rule: d(hinge)/d(z) = -d(sdf)/d(pos) @ d(pos)/d(z)
                J_hinge[i] = -sdf_grad @ jacobians[i, :, :]
            # else: hinge[i] = 0, J_hinge[i] = 0
        
        return hinge, J_hinge
    
    def smooth_hinge_loss(self, z: np.ndarray, 
                          smoothness: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
        """
        Smooth approximation of hinge loss using softplus.
        
        smooth_hinge(d) = smoothness * log(1 + exp(-d / smoothness))
        
        Args:
            z: Quadrotor state [12]
            smoothness: Smoothing parameter (smaller = sharper)
        
        Returns:
            hinge: Smooth hinge loss vector [n_spheres]
            J_hinge: Jacobian [n_spheres x 12]
        """
        eps = self.collision_params.eps_sdf
        
        positions = self.compute_sphere_positions(z)
        jacobians = self.compute_sphere_jacobians(z)
        
        hinge = np.zeros(self.n_spheres)
        J_hinge = np.zeros((self.n_spheres, 12))
        
        for i in range(self.n_spheres):
            pos = positions[i]
            
            sdf_val = self.sdf.query(pos)
            sdf_grad = self.sdf.gradient(pos)
            
            d = sdf_val - self.radii[i] - eps
            
            # Softplus approximation of max(0, -d)
            x = -d / smoothness
            if x > 20:  # Avoid overflow
                hinge[i] = -d
                sigmoid = 1.0
            elif x < -20:  # Avoid underflow
                hinge[i] = 0.0
                sigmoid = 0.0
            else:
                hinge[i] = smoothness * np.log(1 + np.exp(x))
                sigmoid = 1.0 / (1 + np.exp(-x))
            
            # Gradient of softplus: sigmoid(-d/s) * (-1)
            J_hinge[i] = -sigmoid * sdf_grad @ jacobians[i, :, :]
        
        return hinge, J_hinge
    
    def total_collision_cost(self, z: np.ndarray) -> float:
        """
        Compute total weighted collision cost.
        
        Args:
            z: Quadrotor state [12]
        
        Returns:
            cost: Weighted sum of squared hinge losses
        """
        hinge, _ = self.hinge_loss(z)
        sig = self.collision_params.sig_obs
        return sig * np.sum(hinge ** 2)
    
    def collision_cost_trajectory(self, zk: np.ndarray) -> float:
        """
        Compute total collision cost along trajectory.
        
        Args:
            zk: State trajectory [nt x 12]
        
        Returns:
            total_cost: Sum of collision costs
        """
        total = 0.0
        for i in range(zk.shape[0]):
            total += self.total_collision_cost(zk[i])
        return total


def create_obstacle_scene(scene_type: str = "corridor") -> SDF3D:
    """
    Create predefined obstacle scenes for testing.
    
    Args:
        scene_type: One of "corridor", "forest", "room", "single_sphere"
    
    Returns:
        SDF3D object representing the scene
    """
    if scene_type == "single_sphere":
        return SphereSDF(center=np.array([2.0, 0.0, 2.0]), radius=0.5)
    
    elif scene_type == "corridor":
        # Narrow corridor with walls
        obstacles = [
            BoxSDF(center=np.array([2.0, -1.5, 2.0]), half_extents=np.array([3.0, 0.1, 2.0])),
            BoxSDF(center=np.array([2.0, 1.5, 2.0]), half_extents=np.array([3.0, 0.1, 2.0])),
            BoxSDF(center=np.array([2.0, 0.0, 0.0]), half_extents=np.array([3.0, 1.5, 0.1])),  # Floor
            BoxSDF(center=np.array([2.0, 0.0, 4.0]), half_extents=np.array([3.0, 1.5, 0.1])),  # Ceiling
        ]
        return CompositeSDF(obstacles)
    
    elif scene_type == "forest":
        # Random cylinder-like obstacles (approximated as spheres)
        np.random.seed(42)
        n_trees = 10
        obstacles = []
        for _ in range(n_trees):
            x = np.random.uniform(1, 5)
            y = np.random.uniform(-2, 2)
            z = 2.0  # Tree height center
            r = np.random.uniform(0.2, 0.4)
            obstacles.append(SphereSDF(center=np.array([x, y, z]), radius=r))
        return CompositeSDF(obstacles)
    
    elif scene_type == "room":
        # Room with central obstacle
        obstacles = [
            # Walls
            BoxSDF(center=np.array([0.0, 0.0, 2.0]), half_extents=np.array([0.1, 3.0, 2.0])),
            BoxSDF(center=np.array([6.0, 0.0, 2.0]), half_extents=np.array([0.1, 3.0, 2.0])),
            BoxSDF(center=np.array([3.0, -3.0, 2.0]), half_extents=np.array([3.0, 0.1, 2.0])),
            BoxSDF(center=np.array([3.0, 3.0, 2.0]), half_extents=np.array([3.0, 0.1, 2.0])),
            # Central obstacle
            SphereSDF(center=np.array([3.0, 0.0, 2.0]), radius=0.8),
        ]
        return CompositeSDF(obstacles)
    
    else:
        raise ValueError(f"Unknown scene type: {scene_type}")
    
