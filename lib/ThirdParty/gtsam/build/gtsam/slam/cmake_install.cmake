# Install script for directory: /home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam

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
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/AntiFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/BearingFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/BearingRangeFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/BetweenFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/BoundingConstraint.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/EssentialMatrixConstraint.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/EssentialMatrixFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/FrobeniusFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/GeneralSFMFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/InitializePose.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/InitializePose3.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/JacobianFactorQ.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/JacobianFactorQR.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/JacobianFactorSVD.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/KarcherMeanFactor-inl.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/KarcherMeanFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/OrientedPlane3Factor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/PoseRotationPrior.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/PoseTranslationPrior.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/PriorFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/ProjectionFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/RangeFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/ReferenceFrameFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/RegularImplicitSchurFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/RotateFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartFactorBase.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartFactorParams.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartProjectionFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartProjectionPoseFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/SmartProjectionRigFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/StereoFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/TriangulationFactor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/dataset.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/expressions.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/slam/lago.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/build/gtsam/slam/tests/cmake_install.cmake")

endif()

