import numpy as np
import open3d as o3d


def visualize_frames(poses):
    """
    Visualize the world frame and multiple target coordinate frames in Open3D.

    Parameters:
    -----------
    poses : np.ndarray or list of np.ndarray, each shape (4,4)
        One or more homogeneous transformation matrices of target frames relative to the world frame.
    """
    # Normalize input to list
    if isinstance(poses, np.ndarray) and poses.shape == (4, 4):
        pose_list = [poses]
    elif isinstance(poses, list) and all(isinstance(p, np.ndarray) and p.shape == (4, 4) for p in poses):
        pose_list = poses
    else:
        raise ValueError("Input must be a 4x4 numpy array or a list of such arrays.")

    # World frame origin
    world_origin = np.array([0.0, 0.0, 0.0])

    # Create world coordinate frame (size=1.0) and highlight origin with a sphere
    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=world_origin)
    origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    origin_sphere.translate(world_origin)
    origin_sphere.paint_uniform_color([1.0, 1.0, 0.0])  # yellow sphere

    # Prepare list of geometries to draw
    geometries = [world_frame, origin_sphere]

    # Add each target frame
    for idx, pose in enumerate(pose_list):
        # Create smaller coordinate frame for target
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        frame.transform(pose)
        geometries.append(frame)
        # Print target position
        pos = np.round(pose[:3, 3], 4)
        print(f"Target frame {idx+1} position: {pos}")

    # Visualize all frames together
    o3d.visualization.draw_geometries(
        geometries,
        window_name="Coordinate Frames",
        width=800,
        height=600
    )


if __name__ == "__main__":
    # Define pose p1
    p1 = np.array([
        [ 0.05616282,  0.8550789 , -0.5154472 ,  0.88603737],
        [ 0.99776688, -0.0293745 ,  0.05998652, -0.05317714],
        [ 0.03615221, -0.51766516, -0.85481917,  0.43466572],
        [ 0.0       ,  0.0       ,  0.0       ,  1.0       ]
    ])
    # Define pose p2
    p2 = np.array([
        [ 0.9994098,  -0.02532405, -0.02321066, -0.00589255],
        [-0.00696848,  0.51217046, -0.85885555, -0.08404289],
        [ 0.03363752,  0.8585104,   0.51169171,  0.47597559],
        [ 0.0,         0.0,         0.0,         1.0]
    ])
    # Compute composite pose p1 * p2
    p1_p2 = p1 @ p2
    print("Composite pose (p1 * p2) computed.")

    # Visualize p1 and the composite p1*p2
    visualize_frames([p1, p1_p2])
