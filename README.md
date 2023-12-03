# Stochastic motion planning as Gaussian Variational Inference.
This repository is dedicated to implementing Gaussian Variational Inference Motion Planning algorithms (GVIMP). GVIMP was introduced in the following work: 
**[A Gaussian Variational Inference Motion Planning](https://arxiv.org/abs/2209.05655)**, 
and the details and complete proofs are included in the work 
**[Stochastic Motion Planning as Gaussian Variational Inference: Theory and Algorithms](https://arxiv.org/abs/2308.14985)**. 
The latter work also showed the equivalence between the GVI-MP and a classical stochastic control problem. Leveraging the duality between inference and stochastic control, we present another algorithm in the latter paper, namely Proximal Gradient Covariance Steering Motion Planning (PGCS-MP). 


### Point Robot Motion Planning: entropy regularized robust motion planning
<img src="figures/compare_go_through_go_around.png" width="600">

### Motion planning for a 7-DOF WAM robot arm. The animated trajectory is the mean of the trajectory distribution obtained from GVI-MP and PGCS-MP planner, represented by dark gray and silver color, respectively. 

<img src="figures/WAM_GVI_RVIZ_1.gif" width="300" > <img src="figures/WAM_RVIZ_2.gif" width="300">

<img src="figures/WAM_GVI_RVIZ_2.gif" width="300"> <img src="figures/WAM_RVIZ_2.gif" width="300">


**Dependencies**
1. Boost: successful built with Boost1.78.0
Build Boost in a customized location: **[link](https://github.com/hzyu17/technicals/tree/main/C%2B%2B)**
Add library path to ld lib path:
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```
2. Eigen 3.4.0
**[Download Eigen source](https://gitlab.com/libeigen/eigen/-/releases/3.4.0)**
and 
```
cd eigen-3.4.0
mkdir build && cd build\
cmake -DCMAKE_INSTALL_PREFIX=prefix_path ..\
make install 

```
3. Matplot++
**[Matplot++](https://github.com/alandefreitas/matplotplusplus)**

**Build and install VIMP**
```
git clone https://github.com/lucasyu17/VIMP.git
cd VIMP/vimp
mkdir build && cd build\
cmake .. 
sudo make install 
```

**Repository structure**
```
./vimp
├── 3rdparty : dependencies (gpmp2, Eigen, etc.)
├── CMakeLists_customize.txt (A CMakeLists.txt that allows to install to customized location)
├── CMakeLists.txt 
├── configs (Experiment configurations)
├── data 
├── dynamics (Abstract definition of dynamics)
├── gp (Gaussian process related code and definitions)
├── gvimp (Gaussian Variational Inference Motion Planning, abstract definitions)
├── helpers (Some helper classes and functions)
├── instances (Instances of the abstract algorithm definitions with specific Robot Model, Map, and Forward Kinematics)
├── maps (Predefined different maps, mostly borrowed from GPMP2, for the purpose of comparison)
├── pgcsmp (Proximal gradient Covariance Steering Motion Planning, abstract definitions)
├── README.md
├── robots (Robot with Map definitions, for the purpose of instantiate the abstract algorithms.)
├── scripts (ROS related python scripts)
├── src (Experiment executables)
└── tests

./matlab_helpers: This directory reads the results from the ./vimp/src experiment executables, analyze and plot the results.
```

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
