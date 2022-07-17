# remember to add -I/usr/include/python2.7 -lpython2.7 in the cmake options when building
cmake_minimum_required(VERSION 3.16)
project(VIMP CXX C)
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

# add_library(matplotlib SHARED include/matplotlibcpp.h)
# add_library(MVGsampler SHARED include/MVGsampler.h)
# add_library(SparseInverser SHARED include/SparseInverseMatrix.h)
# add_library(SpHelper SHARED include/SparseMatrixHelper.h)
# add_library(FactorizedOptimizer SHARED include/OptimizerFactorized.h)
# add_library(FactorizedOptimizerGaussHermite SHARED include/OptimizerFactorizedGH.h)
# add_library(GaussianProcessPriorLinearPlus SHARED include/GaussianPriorLinearGetQc.h)
# add_library(GaussianHermite SHARED include/GaussHermite.h)
# add_library(FactorizedOptimizerPriorPR SHARED include/OptimizerFactorizedPriorPointRobot.h)


# set_target_properties(matplotlib PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(SpHelper PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(MVGsampler PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(FactorizedOptimizer PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(SparseInverser PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(GaussianProcessPriorLinearPlus PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(GaussianHermite PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(FactorizedOptimizerGaussHermite PROPERTIES LINKER_LANGUAGE CXX)
# set_target_properties(FactorizedOptimizerPriorPR PROPERTIES LINKER_LANGUAGE CXX)


# target_link_libraries(matplotlib ${PYTHON_LIBRARIES})
# target_link_libraries(MVGsampler ${GPMP2_LIBRARIES})
# target_link_libraries(SpHelper ${GPMP2_LIBRARIES})
# target_link_libraries(FactorizedOptimizer ${GPMP2_LIBRARIES})
# target_link_libraries(SparseInverser ${GPMP2_LIBRARIES})
# target_link_libraries(GaussianProcessPriorLinearPlus ${GPMP2_LIBRARIES})
# target_link_libraries(FactorizedOptimizerPriorPR ${GPMP2_LIBRARIES} ${GTSAM_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${PYTHON_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(test_conv src/test_conv.cpp)
target_link_libraries(test_conv ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_conv_factor_Gaussian_GH src/test_conv_factorized_GH.cpp)
target_link_libraries(test_conv_factor_Gaussian_GH ${EIGEN3_LIBRARIES})

add_executable(test_conv_factor src/test_conv_factorized.cpp)
target_link_libraries(test_conv_factor ${PYTHON_LIBRARIES} ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_sparse_inverse src/test_sparse_inverse.cpp)
target_link_libraries(test_sparse_inverse ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_GH src/test_GH.cpp)
target_link_libraries(test_GH ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(test_conv_Gaussian_GH src/test_conv_Gaussian_GH.cpp)
target_link_libraries(test_conv_Gaussian_GH ${EIGEN3_LIBRARIES})

add_executable(test_prior_col_pR src/test_conv_prior_col_pR.cpp)
target_link_libraries(test_prior_col_pR ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(test_prior_pR src/test_conv_prior_pR.cpp)
target_link_libraries(test_prior_pR ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})


# python wrapper
option(VIMP_BUILD_PYTHON "whether build python toolbox, need shared lib" OFF)

if(VIMP_BUILD_PYTHON)
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


# Doxygen
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        COMMAND firefox ${CMAKE_CURRENT_SOURCE_DIR}/html/index.html
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
        
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


