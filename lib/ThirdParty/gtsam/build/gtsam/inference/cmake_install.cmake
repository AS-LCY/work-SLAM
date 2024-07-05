# Install script for directory: /home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference

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
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/BayesNet-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/BayesNet.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/BayesTree-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/BayesTree.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/BayesTreeCliqueBase-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/BayesTreeCliqueBase.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/ClusterTree-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/ClusterTree.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/Conditional-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/Conditional.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/DotWriter.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/EliminateableFactorGraph-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/EliminateableFactorGraph.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/EliminationTree-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/EliminationTree.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/Factor.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/FactorGraph-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/FactorGraph.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/ISAM-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/ISAM.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/JunctionTree-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/JunctionTree.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/Key.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/LabeledSymbol.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/MetisIndex-inl.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/MetisIndex.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/Ordering.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/Symbol.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/VariableIndex-inl.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/VariableIndex.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/VariableSlots.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/graph-inl.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/graph.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/inference-inst.h"
    "/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/gtsam/inference/inferenceExceptions.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/topeet/work/ws_mower/src/lidar_slam/lib/ThirdParty/gtsam/build/gtsam/inference/tests/cmake_install.cmake")

endif()

