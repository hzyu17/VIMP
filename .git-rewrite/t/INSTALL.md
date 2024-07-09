# Installation issues
## 1. Running gtsam matlab wrap
Get the mex-file error
```
Invalid MEX-file '/.../gtsam_toolbox/gtsam_wrapper.mexa64':
/.../gtsam_toolbox/gtsam_wrapper.mexa64: undefined symbol:
_ZTIN5gtsam18Rot3AttitudeFactorE
```
This is a linker's error. It is because the version of libstdc++. A simple resolution is to install a older version of matlab. (Tested success with R2019b).
