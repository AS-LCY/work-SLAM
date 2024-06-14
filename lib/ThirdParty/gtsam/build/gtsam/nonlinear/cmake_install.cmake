# Install script for directory: /home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/nonlinear" TYPE FILE FILES
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/AdaptAutoDiff.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/BatchFixedLagSmoother.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/CustomFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/DoglegOptimizer.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/DoglegOptimizerImpl.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/Expression-inl.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/Expression.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ExpressionFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ExpressionFactorGraph.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ExtendedKalmanFilter-inl.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ExtendedKalmanFilter.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/FixedLagSmoother.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/FunctorizedFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/GaussNewtonOptimizer.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/GncOptimizer.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/GncParams.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/GraphvizFormatting.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ISAM2-impl.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ISAM2.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ISAM2Clique.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ISAM2Params.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ISAM2Result.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/ISAM2UpdateParams.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/LevenbergMarquardtParams.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/LinearContainerFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/Marginals.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/NonlinearEquality.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/NonlinearFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/NonlinearFactorGraph.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/NonlinearISAM.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/NonlinearOptimizer.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/NonlinearOptimizerParams.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/PriorFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/Symbol.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/Values-inl.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/Values.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/WhiteNoiseFactor.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/expressionTesting.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/expressions.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/factorTesting.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/nonlinearExceptions.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/utilities.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/nonlinear/internal" TYPE FILE FILES
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/internal/CallRecord.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/internal/ExecutionTrace.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/internal/ExpressionNode.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/internal/JacobianMap.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/internal/LevenbergMarquardtState.h"
    "/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/nonlinear/internal/NonlinearOptimizerState.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zac/catkin_ws/src/lidar_slam/lib/ThirdParty/gtsam/build/gtsam/nonlinear/tests/cmake_install.cmake")

endif()

