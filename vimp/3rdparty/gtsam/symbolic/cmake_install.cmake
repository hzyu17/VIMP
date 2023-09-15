# Install script for directory: /home/hzyu/git/gtsam/gtsam/symbolic

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/symbolic" TYPE FILE FILES
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicBayesNet.h"
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicBayesTree.h"
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicConditional.h"
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicEliminationTree.h"
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicFactor-inst.h"
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicFactor.h"
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicFactorGraph.h"
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicISAM.h"
    "/home/hzyu/git/gtsam/gtsam/symbolic/SymbolicJunctionTree.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hzyu/git/gtsam/build/gtsam/symbolic/tests/cmake_install.cmake")

endif()

