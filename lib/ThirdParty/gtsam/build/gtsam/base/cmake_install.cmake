# Install script for directory: /home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base

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
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/ConcurrentMap.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/DSFMap.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/DSFVector.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastDefaultAllocator.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastList.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastMap.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastSet.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastVector.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/GenericValue.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Group.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Lie.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Manifold.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Matrix.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/MatrixSerialization.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/OptionalJacobian.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/ProductLieGroup.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/SymmetricBlockMatrix.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Testable.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/TestableAssertions.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/ThreadsafeException.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Value.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Vector.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/VectorSerialization.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/VectorSpace.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/VerticalBlockMatrix.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/WeightedSampler.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/chartTesting.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/cholesky.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/concepts.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/debug.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/kruskal-inl.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/kruskal.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/lieProxies.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/make_shared.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/numericalDerivative.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/serialization.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/serializationTestHelpers.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/std_optional_serialization.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/testLie.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/timing.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/treeTraversal-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/types.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/utilities.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/base/treeTraversal" TYPE FILE FILES
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/treeTraversal/parallelTraversalTasks.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/treeTraversal/statistics.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/build/gtsam/base/tests/cmake_install.cmake")

endif()

