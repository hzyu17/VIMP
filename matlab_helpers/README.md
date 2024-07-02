### Experiment results in the paper: ***'Yu, Hongzhe, and Yongxin Chen. "Stochastic Motion Planning as Gaussian Variational Inference: Theory and Algorithms."*** ' 

#### Map and obstacle environment definitions

##### 1. 2-D maps

1.1 Map for 2D Arm robot motion planning (Fig. 9, 10)
* Origin: [-1, -1] 
* resolution: 0.01
* (width, height): [300, 300]

1.2 Map for 2D point robot motion planning (Fig. 2, 5, 7)
* Origin: [-20, -10] 
* resolution: 0.1
* (width, height): [400, 300]

1.3 Map narrow gap for 2D point robot motion planning (Fig. 1)
* Origin: [-20, -10] 
* resolution: 0.1
* (width, height): [400, 300]

1.4 Map for planar quadrotor motion planning (Fig. 13, 15)
* Origin: [-20, -10] 
* resolution: 0.1
* (width, height): [400, 300]

Detailed obstacle definitions: [See 2D map generation file.](https://github.com/hzyu17/VIMP/blob/paper_experiment_results/matlab_helpers/tools/2dpR/generate2Ddataset_1.m)

##### 2. 3-D maps
2.1 Map for 3D Point robot motion planning (Fig. 8)
* Origin: [-10, -10, -10] 
* resolution: 0.1
* (width, height): [500, 500, 500]

2.2 Map for WAM robot motion planning (Fig. 11, 12)
* Origin: [-1.5, -1.5, -1.5] 
* resolution: 0.01
* (width, height): [300, 300, 300]

Detailed obstacle definitions: [See 3D map generation file.](https://github.com/hzyu17/VIMP/blob/paper_experiment_results/matlab_helpers/tools/generate3Ddataset_1.m)