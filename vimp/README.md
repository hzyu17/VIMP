# Stochastic motion planning as Gaussian Variational Inference.
This repository is dedicated to the implementation of Gaussian Variational Inference (GVI) Motion Planning algorithms. We present 2 algorithms, namely Gaussian Variational Inference Motion Planning (GVI-MP) and Proximal Gradient Covariance Steering Motion Planning (PGCS-MP). The conection between a discrete GVI and continuous-time stochastic control is detailed in the publications in the citing section below.

## Show planned trajectories in ROS environment
1. Install ROS ([noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) as example)
2. An example [repository](https://github.com/hzyu17/moveit_wam_ros_noetic) to create and clone the ROS workspace. 
3. Launch an empty environment
```
roslaunch barrett_wam_moveit_config demo.launch
```
4. Insert example obstacles (bookshelf for example)
```
python pth_to_vimp/scripts/vimp_rosadd_bookshelf.py bookshelf
```
5. Visualize a planned trajectory from a saved csv file
```
python trajectory_displayer_ros_WAM.py
```
6. Reproduce the figures in the paper:
```
python show_trajectories_paper.py --algorithm gpmp2(or)gvi-mp(or)pgcs-mp --exp 1(or)2
```

7. To visualize a voxel grid converted from the point cloud obtained from the camera:
  a. First, click `Add' button in RViz, add `Marker' under `By display type';
  b. Change the topic name to /obstacle_voxel
  c. Publish a voxel message: Check example code: https://github.com/hzyu17/VIMP/blob/cuda/vimp/scripts/vimp_ros/voxel_to_rviz.py

## Citing
If you use this repository in your research, please cite the following publications:
```
@ARTICLE{10068240,
  author={Yu, Hongzhe and Chen, Yongxin},
  journal={IEEE Robotics and Automation Letters}, 
  title={A Gaussian Variational Inference Approach to Motion Planning}, 
  year={2023},
  volume={8},
  number={5},
  pages={2518-2525},
  doi={10.1109/LRA.2023.3256134}}
```
```
@misc{yu2023stochastic,
      title={Stochastic Motion Planning as Gaussian Variational Inference: Theory and Algorithms}, 
      author={Hongzhe Yu and Yongxin Chen},
      year={2023},
      eprint={2308.14985},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
