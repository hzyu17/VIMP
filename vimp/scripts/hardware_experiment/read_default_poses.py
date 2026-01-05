import json, yaml
from vimp.thirdparty.sensor3D_tools import OccpuancyGrid
from pathlib import Path
from geometry_msgs.msg import Pose
import numpy as np

def load_body_frames_config(path: Path):
    """
    Reads a YAML of the form:

    Field:
        field_origin: [0, 0, 0] # [x, y, z] The origin of the field in the april tag's frame
        field_size: [500, 500, 500]
        cell_size: 0.01
        obstacles:
            - name: sdf_box_1
            bodyframe: B1
            center: [0.120, -0.300, 0.050]    # [x, y, z] in metres (wrt frame B)
            orientation: [0.0, 0.0, 0.0, 1.0] # quaternion (qx, qy, qz, qw)
            size: [ 0.18, 0.18, 0.38 ]
      
    body_frames:
      - frame_id: B1
        position:    [x, y, z]
        orientation: [qx, qy, qz, qw]
        monitor_topic: "/B1_pose"
        box:
          name: sdf_box_1
          center: [cx, cy, cz]
          orientation: [qx, qy, qz, qw]
          size: [sx, sy, sz]

    Returns three dicts:
        box_bodyframe: {box_name: bodyframe_id}
        body_topic:    { bodyframe_id: monitor_topic }
        box_rel_poses:  { box_name: Pose() }            # Pose in its body frame
        box_sizes:      { box_name: (sx, sy, sz) }
        body_default_pose: {bodyframe_id: Pose() }
    """
    # Create dictionaries to store the data
    data = yaml.safe_load(path.read_text())
    body_box   = {}
    body_topic = {}
    box_rel_poses = {}
    box_sizes     = {}
    world_box_sizes = {}
    body_default_pose = {}

    # 1) read the pose of the body frames
    for entry in data["Body_frames"]:
        topic = entry["monitor_topic"]
        fname = entry["frame_id"]
        # bname = entry["box"]["name"]
        
        # map box : topic
        body_topic[fname] = topic
        # body_box[fname] = bname
        body_box[fname] = []
        
        # bodyframe : default pose
        p = Pose()
        cx, cy, cz = entry["position"]
        p.position.x, p.position.y, p.position.z = cx, cy, cz
        qx, qy, qz, qw = entry["orientation"]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
            qx, qy, qz, qw
        )
        
        body_default_pose[fname] = p

    # 2) read the box poses
    for entry in data["Field"]["obstacles"]:
        # build a Pose for the box centre in its body frame
        bname = entry["name"]
        frame_id = entry["bodyframe"]
        body_box[frame_id].append(bname)  # map bodyframe to box name
        
        p = Pose()
        p.position.x, p.position.y, p.position.z = entry["center"]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = entry["orientation"]
        box_rel_poses[bname] = p

        # 3) read out the size (you’ll need to add this in your YAML!)
        sx, sy, sz = entry["size"]
        box_sizes[bname] = (sx, sy, sz)
        
    print("body_box")
    print(body_box)
    print("body_topic")
    print(body_topic)
    print("box_rel_poses")
    print(box_rel_poses)
    print("box_sizes")
    print(box_sizes)
    print("body_default_pose")
    print(body_default_pose)

    return body_box, body_topic, box_rel_poses, box_sizes, body_default_pose

def read_body_pose(path: Path):
    """
    Reads the poses from a YAML file of the form
    """
    data = yaml.safe_load(path.read_text())
    body_topic = {}
    body_default_pose = {}

    for entry in data["Body_frames"]:  
        topic = entry["monitor_topic"]
        fname = entry["frame_id"]

        p = Pose()
        p.position.x, p.position.y, p.position.z = entry["position"]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = entry["orientation"]

        body_default_pose[fname] = p
        body_topic[fname] = topic

    return body_default_pose, body_topic


def read_body_tags(path: Path):
    """
    Reads the tag IDs from a YAML file of the form
    """
    data = yaml.safe_load(path.read_text())
    body_topic = {}
    tag_body = {}

    for entry in data["Body_frames"]:  
        topic = entry["monitor_topic"]
        fname = entry["frame_id"]
        tag_id = entry["tag_id"]

        body_topic[fname] = topic
        tag_body[tag_id] = fname

    return body_topic, tag_body


def default_occmap_from_yaml(yaml_file):
    with open(yaml_file,"r") as f:
        cfg = yaml.safe_load(f)
    
    # Create 3D field    
    field = cfg.get("Field", [])
    field_origin = field["field_origin"]
    field_size   = field["field_size"]
    cell_size    = field["cell_size"]
    return OccpuancyGrid(field_size[0], field_size[1], field_size[2], cell_size, origin=field_origin)
    

def read_cam_pose(config_file):
    with open(config_file,"r") as f:
        cfg = yaml.safe_load(f)
    cam_config = cfg["Camera"]["config_file"]
    
    with open(cam_config,"r") as f_cam:
        cam_cfg = json.load(f_cam)
    cam_pose = cam_cfg["pose"]
    
    print("Cam pose:")
    print(cam_pose)
        
    return np.array(cam_pose)
