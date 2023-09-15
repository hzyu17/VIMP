# Install script for directory: /home/hzyu/git/gtsam/gtsam/navigation

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/navigation" TYPE FILE FILES
    "/home/hzyu/git/gtsam/gtsam/navigation/AHRSFactor.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/AttitudeFactor.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/CombinedImuFactor.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/GPSFactor.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/ImuBias.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/ImuFactor.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/MagFactor.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/ManifoldPreintegration.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/NavState.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/PreintegratedRotation.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/PreintegrationBase.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/PreintegrationParams.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/Scenario.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/ScenarioRunner.h"
    "/home/hzyu/git/gtsam/gtsam/navigation/TangentPreintegration.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hzyu/git/gtsam/build/gtsam/navigation/tests/cmake_install.cmake")

endif()

