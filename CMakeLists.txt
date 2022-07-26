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
option(VIMP_BUILD_PYTHON_TOOLBOX "whether build python toolbox, need shared lib" OFF)

if(VIMP_BUILD_STATIC_LIBRARY AND VIMP_BUILD_MATLAB_TOOLBOX)
    message(FATAL_ERROR "matlab toolbox needs shared lib")
endif()

# ##############################################################################
# * Find GTSAM components so we have access to the GTSAM Cython install path
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# ##############################################################################
# * Find Boost components
# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

# ##############################################################################
# * Find gpmp2 components
find_package(gpmp2 REQUIRED)
message(STATUS "gpmp2 DIR: ${gpmp2_DIR}")
set(GPMP2_LIBRARIES gpmp2)
include_directories(${gpmp2_INCLUDE_DIR})

# Generate and install config and dllexport files
configure_file("vimp/config.h.in" "vimp/config.h")
list(APPEND vimp_srcs "${PROJECT_BINARY_DIR}/vimp/config.h")
include_directories(BEFORE ${PROJECT_BINARY_DIR}) # So we can include generated config header files
install(FILES "${PROJECT_BINARY_DIR}/vimp/config.h" DESTINATION include/vimp)

# ##############################################################################
# * Add the local source directory for CMake Ensure that local folder is
#   searched before library folders
# include current source folder, at the very beginning
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

# Process source subdirs
add_subdirectory(vimp)

# ##############################################################################
# Upto this point, we've only focused on building the C++ library. For details
# on how to build the python wrapper, please see the CMakeLists.txt file in the
# `python` directory.
if(VIMP_BUILD_PYTHON_TOOLBOX)
    message(STATUS "VIMP_BUILD_PYTHON_TOOLBOX")
    message(STATUS "GTSAM DIR: ${GTSAM_DIR}")
    message(STATUS "gpmp2 DIR: ${gpmp2_DIR}")
    message(STATUS "GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR}")
    message(STATUS "gpmp2_INCLUDE_DIR: ${gpmp2_INCLUDE_DIR}")
    add_subdirectory(vimp_python)
    # include_directories(${GTSAM_DIR}/cython)
    # include_directories(${gpmp2_DIR}/cython)
    # include_directories(/usr/local/cython)
    # include(GtsamCythonWrap)
    # include_directories(${GTSAM_EIGENCY_INSTALL_PATH})

    # get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    # foreach(dir ${dirs})
    #     message(STATUS "dir='${dir}'")
    # endforeach()

    # wrap_and_install_library_cython("vimp.h"
    #         "from gtsam.gtsam cimport *; from gpmp2.gpmp2 cimport *" # extra import of gtsam/gtsam.pxd Cython header
    #         "${CMAKE_INSTALL_PREFIX}/cython/vimp" # install path
    #         vimp  # library to link with
    #         "gtsam;gpmp2"  # dependencies which need to be built before wrapping
    #         )
    # add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# Install config and export files
GtsamMakeConfigFile(vimp)
export(TARGETS ${vimp_EXPORTED_TARGETS} FILE vimp-exports.cmake)
