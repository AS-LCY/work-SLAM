# Install script for directory: /home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/slam" TYPE FILE FILES
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/AntiFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/BearingFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/BearingRangeFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/BetweenFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/BoundingConstraint.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/EssentialMatrixConstraint.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/EssentialMatrixFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/FrobeniusFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/GeneralSFMFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/InitializePose.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/InitializePose3.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/JacobianFactorQ.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/JacobianFactorQR.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/JacobianFactorSVD.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/KarcherMeanFactor-inl.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/KarcherMeanFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/OrientedPlane3Factor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/PoseRotationPrior.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/PoseTranslationPrior.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/PriorFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/ProjectionFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/RangeFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/ReferenceFrameFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/RegularImplicitSchurFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/RotateFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartFactorBase.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartFactorParams.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartProjectionFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartProjectionPoseFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartProjectionRigFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/StereoFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/TriangulationFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/dataset.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/expressions.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/lago.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/build/gtsam/slam/tests/cmake_install.cmake")

endif()

