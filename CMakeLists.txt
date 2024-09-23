# ----------------------- duplicated contents from the higher level CMakeLists ------------------------
cmake_minimum_required(VERSION 3.0)
enable_testing()
project(VIMP CXX C CUDA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

set(BOOST_ROOT "/usr/local/bin/boost")
# set(BOOST_ROOT "~/.local/bin/boost")
list(APPEND CMAKE_PREFIX_PATH "${BOOST_ROOT}")
# Boost
find_package(Boost 1.70 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)
find_package(Boost COMPONENTS date_time REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# Eigen
message("Finding Eigen3 Library")
find_package( Eigen3 3.4.0 REQUIRED)

# Find the OpenMP package
find_package(OpenMP)

# Set the policy CMP0104 to NEW
if(POLICY CMP0104)
  cmake_policy(SET CMP0104 NEW)
endif()

# Set the CUDA architectures you want to target
set(CMAKE_CUDA_ARCHITECTURES "52;60;61;70;75;80")

# Cuda
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -arch=${CUDA_ARCH}; -std=c++17; --expt-extended-lambda)


# Find Matlab
message("Finding Matlab Library")
# set(MATLAB_ROOT_DIR "~/.local/MATLAB/R2020b")
set(MATLAB_ROOT_DIR "/usr/local/MATLAB/R2020b")
set(Matlab_INCLUDE_DIRS "${MATLAB_ROOT_DIR}/extern/include")
set(Matlab_MEX_LIBRARY "${MATLAB_ROOT_DIR}/bin/glnxa64/libmex.so")  # Adjust the path accordingly
set(MCR_LIBRARY_PATH "${MATLAB_ROOT_DIR}/runtime/glnxa64") 
find_library(Matlab_MX_LIBRARY NAMES libmx.so PATHS "${MATLAB_ROOT_DIR}/bin/glnxa64")

# Sparse GH library
message("Finding Sparse GH Library")
set(libSpGH_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/vimp/GaussianVI/quadrature/libSpGH/for_testing/libSpGH.so")

# GPMP2
message("Finding GPMP2 Library")
set(GPMP2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/gpmp2")
set(GTSAM_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/gtsam")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/vimp")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/vimp/GaussianVI")

include_directories(${EIGEN3_INCLUDE_DIRS})
add_compile_definitions("SOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/vimp")

# To avoid some conflicts and re-definitions that are already existed in GTSAM.
add_compile_definitions(GTSAM_ENV)
add_compile_definitions(GVI_SUBDUR_ENV)

set(GPMP2_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/gpmp2/gpmp2/libgpmp2.so")
set(GTSAM_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/gtsam/gtsam/libgtsam.so.4.0.0")


# matplotcpp
# message("Fetching matplotplusplus Library")
# include(FetchContent)
# FetchContent_Declare(matplotplusplus
#         GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
#         GIT_TAG origin/master) # or whatever tag you want

# FetchContent_GetProperties(matplotplusplus)
# if(NOT matplotplusplus_POPULATED)
#     FetchContent_Populate(matplotplusplus)
#     add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
# endif()

add_subdirectory(vimp)