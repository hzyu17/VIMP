# Stochastic motion planning as Gaussian Variational Inference

This repository is dedicated to implementing Gaussian Variational Inference Motion Planning algorithms (GVI-MP). GVI-MP was introduced in the following work: 
**[A Gaussian Variational Inference Motion Planning](https://arxiv.org/abs/2209.05655)**, 
and the details and complete proofs are included in the work 
**[Stochastic Motion Planning as Gaussian Variational Inference: Theory and Algorithms](https://arxiv.org/abs/2308.14985)**. 
The latter work also showed the equivalence between the GVI-MP and a classical stochastic control problem. Leveraging the duality between inference and stochastic control, we present another algorithm in the latter paper, namely Proximal Covariance Steering Motion Planning (PCS-MP). 

## Planning-as-inferencec
The motion planning problem can be formulated as a probability inference, and the (sub-) optimal trajectory is modeled as a posterior probability $p(X|Z)$, $X$ being the trajecotry, and $Z$ is the environment, often represented by the joint space with obstacles. In this thread of research we use **[GaussianVI](vimp/gvimp/README.md)** to find a Gaussian distribution that is closest to the posterior. We can then sample from the solved Gaussian distribution.

## Examples

**1. Safe and robust motion planning: Entropy maximization formulation**

In a senario with a narrow gap existed between 2 obstacles, classical deterministic motion planner will find a plan that is short, but risky. Our method is equivalently entropy-regularized motion planning. The objective in the stochastic optimal control problem, $\mathbb{E}_q [J(q)]$, is regularized by the entropy of the trajectory joint distribution $H(q)$. The objective we are maximizing is
$$\mathbb{E}_q [J(q)] + H(q).$$

A higher entropy will trade off the short distance risky plan, and gives a longer but safer motion plan. On the right-hand side in the figures below is one illustrative example of this idea.

<img src="figures/Quad_go_through.png" width="300"> <img src="figures/Quad_go_around.png" width="300">


**2. Sampling Trajectory Distributions for Collision Avoidance**

Our method generates a distribution over trajectories rather than a single deterministic one. Therefore, when the environment model is imprecise or unexpected disturbances occur, we can resample from this distribution to obtain a collision‑free trajectory.


<img src="figures/Point_Robot_Resampling_1.gif" width="300"> <img src="figures/Point_Robot_Resampling_2.gif" width="300">

<img src="figures/Point_Robot_Resampling_3.gif" width="300"> <img src="figures/Point_Robot_Resampling_4.gif" width="300">


**3. Variational Mmotion planning for a 7-DOF WAM robot arm**

Our method leverages the factor graph structure of the probabilistic motion planning formulation, and the closed-form expressions for Gaussian posterior expectation computations that does not need other expectation techniques such as Gauss-Hermite quadratures. These structures helped our method to be scalable to higher DOF system such as a industrial robot arm. 

In a bookshelf senario below, the animated trajectory is the mean of the trajectory distribution obtained from GVI-MP and PGCS-MP planner, represented by dark gray and silver color, respectively.

<img src="figures/WAM_GVI_RVIZ_1.gif" width="300" > <img src="figures/WAM_RVIZ_2.gif" width="300">

<img src="figures/WAM_GVI_RVIZ_2.gif" width="300"> <img src="figures/WAM_RVIZ_2.gif" width="300">

**4. Variational Motion planning for a linearized 2D quadrotor (LTV system)**

The experiment settings

<img src="figures/planar_quad_settings.jpg" width="300" >

### Examples
<img src="figures/planar_quad_exp1.gif" height="200" width=180> <img src="figures/planar_quad_exp2.gif" height="200" width=180> 

<img src="figures/planar_quad_exp3.gif" height="200" width=180> <img src="figures/planar_quad_exp4.gif" height="200" width=180>

**5. Hardware Experiments**

Here we compare different algorithms against GVIMP in environments experiencing unexpected disturbances. GVIMP’s ability to resample from the distribution enables obstacle avoidance and increases robustness.

<a href="https://youtu.be/c4sFOlEki0Q?feature=shared">
  <img src="figures/hardware_experiment.gif" width="500" alt="Hardware experiment preview">
</a>

Link to the video: [https://youtu.be/c4sFOlEki0Q?feature=shared](https://youtu.be/c4sFOlEki0Q?feature=shared)

## Dependencies
MATLAB Runtime Library: Download **[MATLAB Runtime R2020b](https://www.mathworks.com/products/compiler/matlab-runtime.html)** and follow **[these instructions](https://www.mathworks.com/help/compiler/install-the-matlab-runtime.html)** to install the source files from the zip file.

## Build and install VIMP

VIMP utilizes a python script to install all necessary dependencies and check system compatibility. All executables can be found in the `VIMP/build/` directory after running the script.
```
git clone https://github.com/hzyu17/VIMP.git
cd VIMP/
python3 build_GVIMP_modules.py
```
## Generate GH-quadratures before running experiments
```
cd build
./src/save_SparseGH_weights
```
## 2D point robot example
To recover the 2D Point robot example in [1, 2], run the following:
```
cd VIMP
git checkout paper_experiment_results
mkdir build && cd build
cmake .. 
make
./src/gvimp/gvi_PointRobot_spgh 
```
The result can be visualized in matlab by running
```
/PathToVIMP/matlab_helpers/GVIMP-examples/2d_pR/planarPR_map2.m
```

## Repository structure
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
If you find this repository useful in your won research, please kindly cite us:
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
```
@misc{chang2024pgvimp,
      title={Accelerating Gaussian Variational Inference for Motion Planning Under Uncertainty}, 
      author={Zinuo Chang and Hongzhe Yu and Patricio Vela and Yongxin Chen},
      year={2024},
      eprint={2411.03416},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
