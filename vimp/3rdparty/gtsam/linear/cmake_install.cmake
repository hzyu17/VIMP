# Install script for directory: /home/hzyu/git/gtsam/gtsam/linear

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/linear" TYPE FILE FILES
    "/home/hzyu/git/gtsam/gtsam/linear/BinaryJacobianFactor.h"
    "/home/hzyu/git/gtsam/gtsam/linear/ConjugateGradientSolver.h"
    "/home/hzyu/git/gtsam/gtsam/linear/Errors.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianBayesNet.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianBayesTree-inl.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianBayesTree.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianConditional-inl.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianConditional.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianDensity.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianEliminationTree.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianFactor.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianFactorGraph.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianISAM.h"
    "/home/hzyu/git/gtsam/gtsam/linear/GaussianJunctionTree.h"
    "/home/hzyu/git/gtsam/gtsam/linear/HessianFactor-inl.h"
    "/home/hzyu/git/gtsam/gtsam/linear/HessianFactor.h"
    "/home/hzyu/git/gtsam/gtsam/linear/IterativeSolver.h"
    "/home/hzyu/git/gtsam/gtsam/linear/JacobianFactor-inl.h"
    "/home/hzyu/git/gtsam/gtsam/linear/JacobianFactor.h"
    "/home/hzyu/git/gtsam/gtsam/linear/KalmanFilter.h"
    "/home/hzyu/git/gtsam/gtsam/linear/NoiseModel.h"
    "/home/hzyu/git/gtsam/gtsam/linear/PCGSolver.h"
    "/home/hzyu/git/gtsam/gtsam/linear/Preconditioner.h"
    "/home/hzyu/git/gtsam/gtsam/linear/RegularHessianFactor.h"
    "/home/hzyu/git/gtsam/gtsam/linear/RegularJacobianFactor.h"
    "/home/hzyu/git/gtsam/gtsam/linear/Sampler.h"
    "/home/hzyu/git/gtsam/gtsam/linear/Scatter.h"
    "/home/hzyu/git/gtsam/gtsam/linear/SubgraphPreconditioner.h"
    "/home/hzyu/git/gtsam/gtsam/linear/SubgraphSolver.h"
    "/home/hzyu/git/gtsam/gtsam/linear/VectorValues.h"
    "/home/hzyu/git/gtsam/gtsam/linear/iterative-inl.h"
    "/home/hzyu/git/gtsam/gtsam/linear/iterative.h"
    "/home/hzyu/git/gtsam/gtsam/linear/linearAlgorithms-inst.h"
    "/home/hzyu/git/gtsam/gtsam/linear/linearExceptions.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hzyu/git/gtsam/build/gtsam/linear/tests/cmake_install.cmake")

endif()

