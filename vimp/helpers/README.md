# Installation issues and guidlines
## When encounter the linking error message 'undefined reference to vtable 'gpmp2::PointRobot...'', it is probably due to the missing of the library files of gtsam or gpmp2. It can be solved by adding the following line to the CMakeLists.txt:
```
set(GTSAM_LIBRARIES "/PREFIX/lib/libgtsam.so.4.0.0")
link_directories(/PREFIX/local/lib/)
set(GPMP2_LIBRARIES "/PREFIX/lib/libgpmp2.so")
link_directories(/PREFIX/local/lib/)
```
where $PREFIX is /usr/local if they are installed using sudo make install, and other known locations if gtsam and gpmp2 are installed to a specific place.

## If matlab is installed in a customized location, then maybe gtsam cannot find <mex.h> if the matlab wrap is built. Then you need to specify the exact location of the mex.h file, which is nomrally stored in MATLAB_PATH/R2020b/extern/include/. Add this in the gtsam CMakeLists.txt:
```
include_directories("MATLAB_PATH/R2020b/extern/include")
```
 
## If build with a customized install location and boost library:
1. GTSAM
```
cmake -DCMAKE_INSTALL_PREFIX=/PREFIX -DGTSAM_BUILD_PYTHON:=OFF -DGTSAM_BUILD_MATLAB:=ON -DGTSAM_INSTALL_MATLAB_TOOLBOX:=ON -DGTSAM_ALLOW_DEPRECATED_SINCE_V4:=ON \
-DGTSAM_INSTALL_CYTHON_TOOLBOX:=OFF  -DGTSAM_USE_SYSTEM_EIGEN:=ON -DCMAKE_PREFIX_PATH=/PREFIX -DBOOST_ROOT=/PREFIX/bin/boost -DBOOST_INCLUDEDIR=/PREFIX/bin/boost/include \
-DBOOST_LIBRARYDIR=/PREFIX/bin/boost/lib -DBoost_NO_BOOST_CMAKE=TRUE -DGTSAM_WITH_EIGEN_MKL=OFF -DGTSAM_WITH_TBB=OFF ..
```
2. GPMP2
```
cmake -DCMAKE_INSTALL_PREFIX=/PREFIX -DGPMP2_BUILD_MATLAB_TOOLBOX:=ON \
-DCMAKE_PREFIX_PATH=/PREFIX -DBOOST_ROOT=/PREFIX/bin/boost \
-DBOOST_INCLUDEDIR=/PREFIX/bin/boost/include \
-DBOOST_LIBRARYDIR=/PREFIX/bin/boost/lib -DBoost_NO_BOOST_CMAKE=TRUE ..
```
