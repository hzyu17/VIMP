import yaml
import numpy as np
from pathlib import Path
import os
from collections import defaultdict
from typing import Any, Dict, List
from bodyframe_pose_listener import pose_to_matrix
from geometry_msgs.msg import Pose

def load_multi_doc(file: Path) -> List[Dict[str, Any]]:
    """Load a YAML file that contains multiple documents separated by `---`."""
    with file.open("r", encoding="utf-8") as f:
        return [doc for doc in yaml.safe_load_all(f) if doc]


def collision_checking(timestamp: np.int64,
                       trajectory_file: str):
    """
    Check for collisions for each trajectory in the baseline planner
    return success rate
    """

    with pose_file.open("r", encoding="utf-8") as f:
        pose = yaml.safe_load(f)


def index_planner_results(planner_docs: List[Dict[str, Any]]) -> Dict[int, List[Dict[str, Any]]]:
    """Group each planner entry under its integer timestamp."""
    idx: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    for doc in planner_docs:
        (ts_str, inner), = doc.items()
        idx[int(ts_str)].append(inner)
    return idx


def gather_results(poses_file: Path,
                   result_file: Path):
    """
    Gather results for each planner
    """
    # Append the results to the lists
    poses_doc = load_multi_doc(poses_file)
    results_doc = load_multi_doc(result_file)

    result_index = index_planner_results(results_doc)

    min_dists: Dict[str, List[float]] = defaultdict(list)

    for pose in poses_doc:
        time_stamp = pose["timestamp"]
        for entry in result_index[time_stamp]:
            planner_id = entry["planner_id"]
            result_key = f"min_dist_{planner_id}"
            if result_key in entry:
                min_dists[planner_id].append(entry[result_key])
            elif planner_id == "GVIMP":
                min_dists[planner_id].append(entry["min_dist_plan"])
            elif planner_id == "GVIMP_Resample":
                min_dists[planner_id].append(entry["min_dist_resample"])

    # Calculate success ratios and create bar plot
    import matplotlib.pyplot as plt

    # Calculate averages
    averages = {}
    min_idx = np.argmin(min_dists["RRTConnect"])
    print("minimum index:", min_idx)
    pose_worse_dict = poses_doc[min_idx]
    print("pose_worse:", pose_worse_dict)
    worst_pose = Pose()
    worst_pose.position.x = pose_worse_dict["position"]["x"]
    worst_pose.position.y = pose_worse_dict["position"]["y"]
    worst_pose.position.z = pose_worse_dict["position"]["z"]
    worst_pose.orientation.x = pose_worse_dict["orientation"]["x"]
    worst_pose.orientation.y = pose_worse_dict["orientation"]["y"]
    worst_pose.orientation.z = pose_worse_dict["orientation"]["z"]
    worst_pose.orientation.w = pose_worse_dict["orientation"]["w"]
    print("worst pose:", worst_pose)

    pose_mat = pose_to_matrix(worst_pose)
    print("worst pose matrix:", pose_mat)

    
    for planner_id, dists in min_dists.items():
        averages[planner_id] = np.mean(dists) if dists else 0

    plt.figure(figsize=(10, 6))
    plt.bar(averages.keys(), averages.values())
    plt.title('Average Minimum Distance by Planner')
    plt.ylabel('Average Distance')
    plt.xlabel('Planner ID')
    plt.show()
        
    # Create scatter plot
    plt.figure(figsize=(10, 6))
    for planner_id, dists in min_dists.items():
        plt.scatter([planner_id] * len(dists), dists, alpha=0.5, label=planner_id)
    plt.title('Minimum Distances by Planner')
    plt.ylabel('Distance')
    plt.xlabel('Planner ID')
    plt.legend()
    plt.show()

    print("Average distances:", averages)

    

if __name__ == "__main__":
    this_dir = os.path.dirname(os.path.abspath(__file__))
    exp_cfg_file = this_dir + "/config/config.yaml"

    pose_file = this_dir + "/Data/Worst_poses.yaml"
    result_file = this_dir + "/Data/hardware_results.yaml"

    gather_results(Path(pose_file), Path(result_file))
        