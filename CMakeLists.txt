# remember to add -I/usr/include/python2.7 -lpython2.7 in the cmake options when building
cmake_minimum_required(VERSION 3.16)
project(MPVI CXX C)
set(CMAKE_CXX_STANDARD 17)

find_package(PythonLibs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gpmp2 REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${GPMP2_INCLUDE_DIRS})
include_directories(${GTSAM_INCLUDE_DIRS})
set(GTSAM_LIBRARIES gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

set(GPMP2_LIBRARIES gpmp2)

#include(Gpmp2MakeConfigFile)
#include(Gpmp2BuildTypes)
#include(Gpmp2Testing)

add_library(matplotlib SHARED include/matplotlibcpp.h)
add_library(MVGsampler SHARED include/MVGsampler.h)
add_library(SparseInverser SHARED include/SparseInverseMatrix.h)
add_library(SpHelper SHARED include/SparseMatrixHelper.h)
add_library(FactorizedOptimizer SHARED include/OptimizerFactorized.h)
add_library(FactorizedOptimizerTwoFactors SHARED include/OptimizerFactorizedTwoFactors.h)
add_library(GaussianProcessPriorLinearPlus SHARED include/GaussianPriorLinearPlus.h)
add_library(GaussianHermite SHARED include/GaussianHermite.h)

set_target_properties(matplotlib PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(SpHelper PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(MVGsampler PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(FactorizedOptimizer PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(FactorizedOptimizerTwoFactors PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(SparseInverser PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(GaussianProcessPriorLinearPlus PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(GaussianHermite PROPERTIES LINKER_LANGUAGE CXX)

target_compile_features(FactorizedOptimizerTwoFactors PUBLIC cxx_std_17)

target_link_libraries(matplotlib ${PYTHON_LIBRARIES})
target_link_libraries(MVGsampler ${GPMP2_LIBRARIES})
target_link_libraries(SpHelper ${GPMP2_LIBRARIES})
target_link_libraries(FactorizedOptimizer ${GPMP2_LIBRARIES})
target_link_libraries(FactorizedOptimizerTwoFactors ${GPMP2_LIBRARIES})
target_link_libraries(SparseInverser ${GPMP2_LIBRARIES})
target_link_libraries(GaussianProcessPriorLinearPlus ${GPMP2_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${PYTHON_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(vimp src/VariationalInferenceMP.cpp)
target_link_libraries(vimp ${PYTHON_LIBRARIES} ${GPMP2_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_conv src/test_convergence.cpp)
target_link_libraries(test_conv ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_point_robot src/test_motion_planning_point_robot.cpp)
target_link_libraries(test_point_robot ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(test_conv_factor_GH src/test_convergence_GH_factorized.cpp)
target_link_libraries(test_conv_factor_GH ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_conv_factor src/test_convergence_factorized.cpp)
target_link_libraries(test_conv_factor ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_sparse_inverse src/test_sparse_inverse.cpp)
target_link_libraries(test_sparse_inverse ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_GH src/test_GH.cpp)
target_link_libraries(test_GH ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

# python wrapper
option(MPVI_BUILD_PYTHON "whether build python toolbox, need shared lib" OFF)

if(MPVI_BUILD_PYTHON)
#    include_directories(${GTSAM_DIR}/cython)
#    include_directories(/usr/local/cython)
#    include(GtsamCythonWrap)
#    include_directories(${GTSAM_EIGENCY_INSTALL_PATH})
#
#    wrap_and_install_library_cython("gpmp2.h"
#            "from gtsam.gtsam cimport *" # extra import of gtsam/gtsam.pxd Cython header
#            "${CMAKE_INSTALL_PREFIX}/cython" # install path
#            gpmp2  # library to link with
#            "gtsam"  # dependencies which need to be built before wrapping
#            )
#    add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

install(TARGETS matplotlib MVGsampler SpHelper
        CONFIGURATIONS Release
        LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

