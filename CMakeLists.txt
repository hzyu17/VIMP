# ----------------------- duplicated contents from the higher level CMakeLists ------------------------
cmake_minimum_required(VERSION 3.0)
enable_testing()
project(VIMP CXX C)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

set(BOOST_ROOT "/usr/local/bin/boost")
list(APPEND CMAKE_PREFIX_PATH "${BOOST_ROOT}")
# Boost - same requirement as gtsam
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
find_package( Eigen3 3.4.0 REQUIRED)

set(GPMP2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/gpmp2")
set(GTSAM_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/gtsam")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/vimp")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/vimp/GaussianVI")

include_directories(${EIGEN3_INCLUDE_DIRS})
add_compile_definitions("SOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/vimp")

set(GPMP2_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/gpmp2/gpmp2/libgpmp2.so")
set(GTSAM_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/vimp/3rdparty/gtsam/gtsam/libgtsam.so.4.0.0")

 
# matplotcpp
include(FetchContent)
FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master) # or whatever tag you want

FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

add_subdirectory(vimp)