# Install script for directory: /home/hzyu/git/gtsam/gtsam/base

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/base" TYPE FILE FILES
    "/home/hzyu/git/gtsam/gtsam/base/ConcurrentMap.h"
    "/home/hzyu/git/gtsam/gtsam/base/DSFVector.h"
    "/home/hzyu/git/gtsam/gtsam/base/DerivedValue.h"
    "/home/hzyu/git/gtsam/gtsam/base/FastDefaultAllocator.h"
    "/home/hzyu/git/gtsam/gtsam/base/FastList.h"
    "/home/hzyu/git/gtsam/gtsam/base/FastMap.h"
    "/home/hzyu/git/gtsam/gtsam/base/FastSet.h"
    "/home/hzyu/git/gtsam/gtsam/base/FastVector.h"
    "/home/hzyu/git/gtsam/gtsam/base/GenericValue.h"
    "/home/hzyu/git/gtsam/gtsam/base/Group.h"
    "/home/hzyu/git/gtsam/gtsam/base/Lie.h"
    "/home/hzyu/git/gtsam/gtsam/base/LieMatrix.h"
    "/home/hzyu/git/gtsam/gtsam/base/LieScalar.h"
    "/home/hzyu/git/gtsam/gtsam/base/LieVector.h"
    "/home/hzyu/git/gtsam/gtsam/base/Manifold.h"
    "/home/hzyu/git/gtsam/gtsam/base/Matrix.h"
    "/home/hzyu/git/gtsam/gtsam/base/OptionalJacobian.h"
    "/home/hzyu/git/gtsam/gtsam/base/ProductLieGroup.h"
    "/home/hzyu/git/gtsam/gtsam/base/SymmetricBlockMatrix.h"
    "/home/hzyu/git/gtsam/gtsam/base/Testable.h"
    "/home/hzyu/git/gtsam/gtsam/base/TestableAssertions.h"
    "/home/hzyu/git/gtsam/gtsam/base/ThreadsafeException.h"
    "/home/hzyu/git/gtsam/gtsam/base/Value.h"
    "/home/hzyu/git/gtsam/gtsam/base/Vector.h"
    "/home/hzyu/git/gtsam/gtsam/base/VectorSpace.h"
    "/home/hzyu/git/gtsam/gtsam/base/VerticalBlockMatrix.h"
    "/home/hzyu/git/gtsam/gtsam/base/chartTesting.h"
    "/home/hzyu/git/gtsam/gtsam/base/cholesky.h"
    "/home/hzyu/git/gtsam/gtsam/base/concepts.h"
    "/home/hzyu/git/gtsam/gtsam/base/debug.h"
    "/home/hzyu/git/gtsam/gtsam/base/lieProxies.h"
    "/home/hzyu/git/gtsam/gtsam/base/numericalDerivative.h"
    "/home/hzyu/git/gtsam/gtsam/base/serialization.h"
    "/home/hzyu/git/gtsam/gtsam/base/serializationTestHelpers.h"
    "/home/hzyu/git/gtsam/gtsam/base/testLie.h"
    "/home/hzyu/git/gtsam/gtsam/base/timing.h"
    "/home/hzyu/git/gtsam/gtsam/base/treeTraversal-inst.h"
    "/home/hzyu/git/gtsam/gtsam/base/types.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/base/treeTraversal" TYPE FILE FILES
    "/home/hzyu/git/gtsam/gtsam/base/treeTraversal/parallelTraversalTasks.h"
    "/home/hzyu/git/gtsam/gtsam/base/treeTraversal/statistics.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/base/deprecated" TYPE FILE FILES
    "/home/hzyu/git/gtsam/gtsam/base/deprecated/LieMatrix.h"
    "/home/hzyu/git/gtsam/gtsam/base/deprecated/LieScalar.h"
    "/home/hzyu/git/gtsam/gtsam/base/deprecated/LieVector.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hzyu/git/gtsam/build/gtsam/base/tests/cmake_install.cmake")

endif()

