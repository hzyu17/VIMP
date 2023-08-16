# Stochastic motion planning as Gaussian Variational Inference.
This repository is dedicated to the implementation of Gaussian Variational Inference (GVI) Motion Planning algorithms. We present 2 algorithms, namely Gaussian Variational Inference Motion Planning (GVI-MP) and Proximal Gradient Covariance Steering Motion Planning (PGCS-MP). The conection between a discrete GVI and continuous-time stochastic control is detailed in [1], [2].

## Show planned trajectories in ROS environment
1. Install ROS ([noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) as example)
2. Follow this [repository](https://github.com/hzyu17/moveit_wam_ros_noetic) README to create and clone the ROS workspace. 
3. Launch an empty environment
```
roslaunch barrett_wam_moveit_config demo.launch
```
4. Insert example obstacles (bookshelf for example)
```
cd vimp/scripts
python add_bookshelf.py bookshelf
```
5. Visualize a planned trajectory from a saved csv file
```
python test_display_trajectory_wam.py
```


## Citation
If you use this repository in your research, please cite the following publications
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
