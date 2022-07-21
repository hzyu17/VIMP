cmake_minimum_required(VERSION 3.0)
enable_testing()
project(vimp CXX C)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
    set(CMAKE_MACOSX_RPATH 0)
endif()

# version indicator
set(VIMP_VERSION_MAJOR 0)
set(VIMP_VERSION_MINOR 0)
set(VIMP_VERSION_PATCH 1)
set(vimp_VERSION_STRING "${VIMP_VERSION_MAJOR}.${VIMP_VERSION_MINOR}.${VIMP_VERSION_PATCH}")

# option: whether turn on Matlab toolbox
option(VIMP_BUILD_STATIC_LIBRARY "whether build static library" OFF)
option(VIMP_BUILD_MATLAB_TOOLBOX "whether build matlab toolbox, need shared lib" OFF)
option(VIMP_BUILD_PYTHON_TOOLBOX "whether build python toolbox, need shared lib" ON)

if(VIMP_BUILD_STATIC_LIBRARY AND VIMP_BUILD_MATLAB_TOOLBOX)
    message(FATAL_ERROR "matlab toolbox needs shared lib")
endif()

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

# Generate and install config and dllexport files
#configure_file("vimp/config.h.in" "vimp/config.h")
# list(APPEND vimp_srcs "${PROJECT_BINARY_DIR}/vimp/config.h")
# include_directories(BEFORE ${PROJECT_BINARY_DIR}) # So we can include generated config header files
# install(FILES "${PROJECT_BINARY_DIR}/vimp/config.h" DESTINATION include/vimp)

# include current source folder, at the very beginning
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

# Process source subdirs
add_subdirectory(vimp)

# Wrapping to MATLAB
if(VIMP_BUILD_MATLAB_TOOLBOX)
    # wrap
    include(GtsamMatlabWrap)
    wrap_and_install_library(VIMP.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")

    # install matlab functions and scripts
    add_subdirectory(matlab)
endif()

# Wrapping to Python
if(VIMP_BUILD_PYTHON_TOOLBOX)
    message(STATUS "VIMP_BUILD_PYTHON_TOOLBOX")
    include_directories(${GTSAM_DIR}/cython)
    include_directories(/usr/local/cython)
    include(GtsamCythonWrap)
    include_directories(${GTSAM_EIGENCY_INSTALL_PATH})

    wrap_and_install_library_cython("vimp.h"
            "from gtsam.gtsam cimport *" # extra import of gtsam/gtsam.pxd Cython header
            "${CMAKE_INSTALL_PREFIX}/cython" # install path
            vimp  # library to link with
            "gtsam"  # dependencies which need to be built before wrapping
            )
    add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# Install config and export files
GtsamMakeConfigFile(vimp)
# export(TARGETS ${vimp_EXPORTED_TARGETS} FILE vimp-exports.cmake)
