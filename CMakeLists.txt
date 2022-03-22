# remember to add -I/usr/include/python2.7 -lpython2.7 in the cmake options when building
cmake_minimum_required(VERSION 3.16)
project(MPVI CXX C)
set(CMAKE_CXX_STANDARD 14)

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
add_library(FactorizedOptimizerTwoByTwo SHARED include/OptimizerTwoByTwo.h)

set_target_properties(matplotlib PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(SpHelper PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(MVGsampler PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(FactorizedOptimizerTwoByTwo PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(SparseInverser PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(matplotlib ${PYTHON_LIBRARIES})
target_link_libraries(MVGsampler ${GPMP2_LIBRARIES})
target_link_libraries(SpHelper ${GPMP2_LIBRARIES})
target_link_libraries(FactorizedOptimizerTwoByTwo ${GPMP2_LIBRARIES})
target_link_libraries(SparseInverser ${GPMP2_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${PYTHON_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(vimp src/VariationalInferenceMP.cpp)
target_link_libraries(vimp ${PYTHON_LIBRARIES} ${GPMP2_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_conv src/test_convergence.cpp)
target_link_libraries(test_conv ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

#add_executable(test_mp_point_robot src/test_motion_planning_point_robot.cpp)
#target_link_libraries(test_mp_point_robot ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(test_conv_factor src/test_convergence_factorized.cpp)
target_link_libraries(test_conv_factor ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_sparse_inverse src/test_sparse_inverse.cpp)
target_link_libraries(test_sparse_inverse ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

install(TARGETS matplotlib MVGsampler SpHelper
        CONFIGURATIONS Release
        LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)