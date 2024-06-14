# Install script for directory: /home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base

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
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/ConcurrentMap.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/DSFMap.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/DSFVector.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastDefaultAllocator.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastList.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastMap.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastSet.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/FastVector.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/GenericValue.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Group.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Lie.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Manifold.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Matrix.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/MatrixSerialization.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/OptionalJacobian.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/ProductLieGroup.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/SymmetricBlockMatrix.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Testable.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/TestableAssertions.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/ThreadsafeException.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Value.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/Vector.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/VectorSerialization.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/VectorSpace.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/VerticalBlockMatrix.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/WeightedSampler.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/chartTesting.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/cholesky.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/concepts.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/debug.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/kruskal-inl.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/kruskal.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/lieProxies.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/make_shared.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/numericalDerivative.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/serialization.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/serializationTestHelpers.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/std_optional_serialization.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/testLie.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/timing.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/treeTraversal-inst.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/types.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/utilities.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/base/treeTraversal" TYPE FILE FILES
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/treeTraversal/parallelTraversalTasks.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/base/treeTraversal/statistics.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/build/gtsam/base/tests/cmake_install.cmake")

endif()

