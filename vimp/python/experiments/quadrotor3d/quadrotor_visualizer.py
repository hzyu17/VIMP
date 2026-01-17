#!/usr/bin/env python3
"""
load_trajectory_example.py

Example: Load a binary trajectory file saved from C++ and visualize it.

Usage:
    python load_trajectory_example.py figure8_trajectory.traj
"""

import numpy as np
import struct
import sys
from pathlib import Path

# ==================== Binary I/O (standalone version) ====================

def load_trajectory_binary(filepath: str) -> dict:
    """
    Load trajectory from .traj binary format.
    
    Returns:
        dict with keys: 'trajectory', 'dt', 'total_time', 'n_states', 'dim_state'
    """
    with open(filepath, 'rb') as f:
        # Read magic number
        magic = f.read(4)
        if magic != b'TRAJ':
            raise ValueError(f"Invalid file format. Expected 'TRAJ', got {magic}")
        
        # Read header: version, n_states, dim_state
        version, n_states, dim_state = struct.unpack('<III', f.read(12))
        print(f"File version: {version}")
        print(f"Shape: {n_states} states x {dim_state} dimensions")
        
        # Read trajectory data (row-major doubles)
        n_bytes = n_states * dim_state * 8  # 8 bytes per double
        data = f.read(n_bytes)
        trajectory = np.frombuffer(data, dtype=np.float64).reshape(n_states, dim_state)
        
        # Read metadata (version 2+)
        dt, total_time = 0.0, 0.0
        if version >= 2:
            meta = f.read(16)
            if len(meta) == 16:
                dt, total_time = struct.unpack('<dd', meta)
        
        return {
            'trajectory': trajectory.copy(),  # Copy to make it writable
            'dt': dt,
            'total_time': total_time,
            'n_states': n_states,
            'dim_state': dim_state
        }


def print_trajectory_info(data: dict):
    """Print trajectory statistics."""
    traj = data['trajectory']
    
    print(f"\n{'='*50}")
    print("Trajectory Information")
    print(f"{'='*50}")
    print(f"Number of states: {data['n_states']}")
    print(f"State dimension:  {data['dim_state']}")
    print(f"Time step (dt):   {data['dt']:.4f} s")
    print(f"Total time:       {data['total_time']:.2f} s")
    
    print(f"\nPosition bounds:")
    print(f"  X: [{traj[:,0].min():.3f}, {traj[:,0].max():.3f}] m")
    print(f"  Y: [{traj[:,1].min():.3f}, {traj[:,1].max():.3f}] m")
    print(f"  Z: [{traj[:,2].min():.3f}, {traj[:,2].max():.3f}] m")
    
    print(f"\nAttitude bounds (degrees):")
    print(f"  Roll (φ):  [{np.degrees(traj[:,3]).min():.2f}, {np.degrees(traj[:,3]).max():.2f}]°")
    print(f"  Pitch (θ): [{np.degrees(traj[:,4]).min():.2f}, {np.degrees(traj[:,4]).max():.2f}]°")
    print(f"  Yaw (ψ):   [{np.degrees(traj[:,5]).min():.2f}, {np.degrees(traj[:,5]).max():.2f}]°")
    
    print(f"\nFirst state:")
    print(f"  Position: ({traj[0,0]:.3f}, {traj[0,1]:.3f}, {traj[0,2]:.3f})")
    print(f"  Attitude: ({np.degrees(traj[0,3]):.1f}°, {np.degrees(traj[0,4]):.1f}°, {np.degrees(traj[0,5]):.1f}°)")
    
    print(f"\nLast state:")
    print(f"  Position: ({traj[-1,0]:.3f}, {traj[-1,1]:.3f}, {traj[-1,2]:.3f})")
    print(f"  Attitude: ({np.degrees(traj[-1,3]):.1f}°, {np.degrees(traj[-1,4]):.1f}°, {np.degrees(traj[-1,5]):.1f}°)")


def plot_trajectory_2d(data: dict):
    """Plot 2D projections of the trajectory."""
    import matplotlib.pyplot as plt
    
    traj = data['trajectory']
    t = np.linspace(0, data['total_time'], data['n_states'])
    
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    
    # XY plot (top view)
    axes[0, 0].plot(traj[:, 0], traj[:, 1], 'b-', lw=2)
    axes[0, 0].plot(traj[0, 0], traj[0, 1], 'go', ms=10, label='Start')
    axes[0, 0].plot(traj[-1, 0], traj[-1, 1], 'ro', ms=10, label='End')
    axes[0, 0].set_xlabel('X [m]')
    axes[0, 0].set_ylabel('Y [m]')
    axes[0, 0].set_title('Top View (XY)')
    axes[0, 0].axis('equal')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    # XZ plot (side view)
    axes[0, 1].plot(traj[:, 0], traj[:, 2], 'b-', lw=2)
    axes[0, 1].set_xlabel('X [m]')
    axes[0, 1].set_ylabel('Z [m]')
    axes[0, 1].set_title('Side View (XZ)')
    axes[0, 1].grid(True, alpha=0.3)
    
    # YZ plot (front view)
    axes[0, 2].plot(traj[:, 1], traj[:, 2], 'b-', lw=2)
    axes[0, 2].set_xlabel('Y [m]')
    axes[0, 2].set_ylabel('Z [m]')
    axes[0, 2].set_title('Front View (YZ)')
    axes[0, 2].grid(True, alpha=0.3)
    
    # Position vs time
    axes[1, 0].plot(t, traj[:, 0], 'r-', label='X')
    axes[1, 0].plot(t, traj[:, 1], 'g-', label='Y')
    axes[1, 0].plot(t, traj[:, 2], 'b-', label='Z')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Position [m]')
    axes[1, 0].set_title('Position vs Time')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Attitude vs time
    axes[1, 1].plot(t, np.degrees(traj[:, 3]), 'r-', label='Roll (φ)')
    axes[1, 1].plot(t, np.degrees(traj[:, 4]), 'g-', label='Pitch (θ)')
    axes[1, 1].plot(t, np.degrees(traj[:, 5]), 'b-', label='Yaw (ψ)')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Angle [deg]')
    axes[1, 1].set_title('Attitude vs Time')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    # Velocity vs time
    if data['dim_state'] >= 9:
        axes[1, 2].plot(t, traj[:, 6], 'r-', label='vx')
        axes[1, 2].plot(t, traj[:, 7], 'g-', label='vy')
        axes[1, 2].plot(t, traj[:, 8], 'b-', label='vz')
        axes[1, 2].set_xlabel('Time [s]')
        axes[1, 2].set_ylabel('Velocity [m/s]')
        axes[1, 2].set_title('Body Velocity vs Time')
        axes[1, 2].legend()
        axes[1, 2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectory_plots.png', dpi=150)
    print("\nSaved 2D plots to: trajectory_plots.png")
    plt.show()


def plot_trajectory_3d(data: dict):
    """Plot 3D trajectory."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    traj = data['trajectory']
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot trajectory
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'b-', lw=2, label='Trajectory')
    
    # Mark start and end
    ax.scatter(*traj[0, :3], c='green', s=100, marker='o', label='Start')
    ax.scatter(*traj[-1, :3], c='red', s=100, marker='s', label='End')
    
    # Plot some intermediate positions with attitude indicators
    n_arrows = 10
    indices = np.linspace(0, len(traj)-1, n_arrows, dtype=int)
    
    for idx in indices:
        pos = traj[idx, :3]
        phi, theta, psi = traj[idx, 3:6]
        
        # Simple body-x direction indicator
        dx = 0.3 * np.cos(psi) * np.cos(theta)
        dy = 0.3 * np.sin(psi) * np.cos(theta)
        dz = -0.3 * np.sin(theta)
        
        ax.quiver(pos[0], pos[1], pos[2], dx, dy, dz, 
                  color='red', alpha=0.6, arrow_length_ratio=0.3)
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D Quadrotor Trajectory')
    ax.legend()
    
    # Equal aspect ratio
    max_range = np.array([
        traj[:, 0].max() - traj[:, 0].min(),
        traj[:, 1].max() - traj[:, 1].min(),
        traj[:, 2].max() - traj[:, 2].min()
    ]).max() / 2.0
    
    mid_x = (traj[:, 0].max() + traj[:, 0].min()) / 2
    mid_y = (traj[:, 1].max() + traj[:, 1].min()) / 2
    mid_z = (traj[:, 2].max() + traj[:, 2].min()) / 2
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    plt.savefig('trajectory_3d.png', dpi=150)
    print("Saved 3D plot to: trajectory_3d.png")
    plt.show()


# ==================== Main ====================

def main():
    if len(sys.argv) < 2:
        print("Usage: python load_trajectory_example.py <trajectory.traj>")
        print("\nTo generate a test file, first compile and run the C++ example:")
        print("  g++ -std=c++17 -O2 save_trajectory_example.cpp -o save_trajectory -I/usr/include/eigen3")
        print("  ./save_trajectory test.traj")
        print("  python load_trajectory_example.py test.traj")
        sys.exit(1)
    
    filepath = sys.argv[1]
    
    if not Path(filepath).exists():
        print(f"Error: File not found: {filepath}")
        sys.exit(1)
    
    # Load trajectory
    print(f"Loading: {filepath}")
    print(f"File size: {Path(filepath).stat().st_size:,} bytes")
    
    data = load_trajectory_binary(filepath)
    
    # Print info
    print_trajectory_info(data)
    
    # Plot
    try:
        import matplotlib
        print("\n" + "="*50)
        print("Plotting...")
        plot_trajectory_2d(data)
        plot_trajectory_3d(data)
    except ImportError:
        print("\nMatplotlib not available. Install with: pip install matplotlib")
    except Exception as e:
        print(f"\nCould not display plots: {e}")
        print("Plots saved to trajectory_plots.png and trajectory_3d.png")


if __name__ == '__main__':
    main()
