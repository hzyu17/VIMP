# Install script for directory: /home/hzyu/git/gtsam/gtsam/inference

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/inference" TYPE FILE FILES
    "/home/hzyu/git/gtsam/gtsam/inference/BayesNet-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/BayesNet.h"
    "/home/hzyu/git/gtsam/gtsam/inference/BayesTree-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/BayesTree.h"
    "/home/hzyu/git/gtsam/gtsam/inference/BayesTreeCliqueBase-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/BayesTreeCliqueBase.h"
    "/home/hzyu/git/gtsam/gtsam/inference/ClusterTree-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/ClusterTree.h"
    "/home/hzyu/git/gtsam/gtsam/inference/Conditional-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/Conditional.h"
    "/home/hzyu/git/gtsam/gtsam/inference/EliminateableFactorGraph-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/EliminateableFactorGraph.h"
    "/home/hzyu/git/gtsam/gtsam/inference/EliminationTree-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/EliminationTree.h"
    "/home/hzyu/git/gtsam/gtsam/inference/Factor.h"
    "/home/hzyu/git/gtsam/gtsam/inference/FactorGraph-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/FactorGraph.h"
    "/home/hzyu/git/gtsam/gtsam/inference/ISAM-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/ISAM.h"
    "/home/hzyu/git/gtsam/gtsam/inference/JunctionTree-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/JunctionTree.h"
    "/home/hzyu/git/gtsam/gtsam/inference/Key.h"
    "/home/hzyu/git/gtsam/gtsam/inference/LabeledSymbol.h"
    "/home/hzyu/git/gtsam/gtsam/inference/MetisIndex-inl.h"
    "/home/hzyu/git/gtsam/gtsam/inference/MetisIndex.h"
    "/home/hzyu/git/gtsam/gtsam/inference/Ordering.h"
    "/home/hzyu/git/gtsam/gtsam/inference/Symbol.h"
    "/home/hzyu/git/gtsam/gtsam/inference/VariableIndex-inl.h"
    "/home/hzyu/git/gtsam/gtsam/inference/VariableIndex.h"
    "/home/hzyu/git/gtsam/gtsam/inference/VariableSlots.h"
    "/home/hzyu/git/gtsam/gtsam/inference/graph-inl.h"
    "/home/hzyu/git/gtsam/gtsam/inference/graph.h"
    "/home/hzyu/git/gtsam/gtsam/inference/inference-inst.h"
    "/home/hzyu/git/gtsam/gtsam/inference/inferenceExceptions.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hzyu/git/gtsam/build/gtsam/inference/tests/cmake_install.cmake")

endif()

