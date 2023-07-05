
# Gaussian variational inference (GVI) motion planning.

![Planning using Gaussian variational inference (GVI)](comparison.png)
VIMP uses Gaussian variational inference to approach the motion planning problem as a probability inference. This project is built upon GTSAM and GPMP2.

**Build Eigen 3.4.0**
**[Download Eigen source](https://gitlab.com/libeigen/eigen/-/releases/3.4.0)**
and 
```
cd eigen-3.4.0
mkdir build && cd build\
cmake -DCMAKE_INSTALL_PREFIX=prefix_path ..\
make install 

```
 
**Build GTSAM**
**[GTSAM](https://github.com/borglab/gtsam)**
 Build and installation:
   ```
   git clone https://github.com/borglab/gtsam.git 
   cd gtsam && mkdir build && cd build
   cmake -DGTSAM_BUILD_PYTHON:=OFF -DGTSAM_BUILD_MATLAB:=ON -DGTSAM_INSTALL_MATLAB_TOOLBOX:=ON -DGTSAM_ALLOW_DEPRECATED_SINCE_V4:=OFF -DGTSAM_INSTALL_CYTHON_TOOLBOX:=OFF  -DGTSAM_USE_SYSTEM_EIGEN:=ON -DGTSAM_BUILD_UNSTABLE:=OFF
   -DGTSAM_WITH_TBB:=OFF .. 
   sudo make install
   ```

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
