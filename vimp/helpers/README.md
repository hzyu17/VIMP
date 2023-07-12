# Installation issues and guidlines
When encounter the linking error message 'undefined reference to vtable 'gpmp2::PointRobot...'', it is probably due to the missing of the library files of gtsam or gpmp2. It can be solved by adding the following line to the CMakeLists.txt:
```
set(GTSAM_LIBRARIES "/PREFIX/lib/libgtsam.so.4.0.0")
link_directories(/PREFIX/local/lib/)
set(GPMP2_LIBRARIES "/PREFIX/lib/libgpmp2.so")
link_directories(/PREFIX/local/lib/)
```
where $PREFIX is /usr/local if they are installed using sudo make install, and other known locations if gtsam and gpmp2 are installed to a specific place.

