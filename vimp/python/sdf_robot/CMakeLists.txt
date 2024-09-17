cmake_minimum_required(VERSION 3.12)
project(SDFLibrary)

# Find Python and pybind11
find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

# Set the path to Eigen
set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")

# Add your C++ source file
add_library(sdf MODULE src/SignedDistanceField.cpp)
add_library(planar_sdf MODULE src/PlanarSDF.cpp)
target_link_libraries(sdf PUBLIC pybind11::module ${Python_LIBRARIES})
target_link_libraries(planar_sdf PUBLIC pybind11::module ${Python_LIBRARIES})

# Include directories for pybind11 and Eigen
target_include_directories(sdf PRIVATE ${Python_INCLUDE_DIRS} ${pybind11_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
target_include_directories(planar_sdf PRIVATE ${Python_INCLUDE_DIRS} ${pybind11_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
