
# Gaussian variational inference (GVI) motion planning.

![Planning using Gaussian variational inference (GVI)](comparison.png)
VIMP uses Gaussian variational inference to approach the motion planning problem as a probability inference. This project is built upon GTSAM and GPMP2.

**Dependencies**
1. Boost: successful built with Boost1.78.0
Build Boost in a customized location: **[link](https://github.com/hzyu17/technicals/tree/main/C%2B%2B)**
2. Eigen 3.4.0
**[Download Eigen source](https://gitlab.com/libeigen/eigen/-/releases/3.4.0)**
and 
```
cd eigen-3.4.0
mkdir build && cd build\
cmake -DCMAKE_INSTALL_PREFIX=prefix_path ..\
make install 

```
3. GTSAM 
**Build GTSAM**
**[GTSAM](https://github.com/borglab/gtsam)**
 Build and installation:
 ```
 git clone https://github.com/borglab/gtsam.git 
 cd gtsam
 git caheckout wrap-export
 mkdir build && cd build
 cmake -DGTSAM_BUILD_PYTHON:=OFF -DGTSAM_BUILD_MATLAB:=ON -DGTSAM_INSTALL_MATLAB_TOOLBOX:=ON -DGTSAM_ALLOW_DEPRECATED_SINCE_V4:=OFF -DGTSAM_INSTALL_CYTHON_TOOLBOX:=OFF  -DGTSAM_USE_SYSTEM_EIGEN:=ON -DGTSAM_BUILD_UNSTABLE:=OFF
 -DGTSAM_WITH_TBB:=OFF .. 
 sudo make install
 ```
4. GPMP2
**Build GPMP2**
**[GPMP2](https://github.com/gtrll/gpmp2)**
Build and installation:
 ```
 git clone https://github.com/gtrll/gpmp2.git
 cd gpmp2 && mkdir build && cd build\
 cmake -DGPMP2_BUILD_PYTHON_TOOLBOX:=OFF -DGPMP2_BUILD_MATLAB_TOOLBOX:=ON .. 
 sudo make install 
 ```

**Build and install VIMP**
```
git clone https://github.com/lucasyu17/VIMP.git
mkdir build && cd build\
cmake .. 
sudo make install 
```
