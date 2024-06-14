cmake_minimum_required(VERSION 2.8.3)
project(lidar_slam)

SET(CMAKE_BUILD_TYPE "Release")  
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_compile_options(-fPIC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

get_filename_component(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
add_compile_definitions(CURRENT_DIR="${CURRENT_DIR}")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(Boost 1.54 REQUIRED COMPONENTS
  system
  thread
  chrono
)

find_package( FMT REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
#find_package (GeographicLib REQUIRED)
find_package(OpenCV REQUIRED QUIET)
find_package( GTSAMCMakeTools )
find_package(GTSAM REQUIRED QUIET)
find_package( Sophus REQUIRED )
find_package(Pangolin REQUIRED)
# find_package(fastcdr REQUIRED)
# find_package(fastrtps REQUIRED)
find_package(yaml-cpp REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})
find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif (APR_FOUND)

  find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    /usr/local/lib)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
 # ${GeographicLib_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  include
  ThirdParty/fast_gicp/include
  ThirdParty/Livox-SDK2/include
  ${APR_INCLUDE_DIRS}
  ThirdParty/livox_ros_driver2/3rdparty
  ThirdParty/livox_ros_driver2/src)


  # link directories
 link_directories(
#   ${GTSAM_LIBRARY_DIRS}
   ${OpenCV_LIBRARY_DIRS}
   ThirdParty/fast_gicp/build/devel/lib/
   ThirdParty/Livox-SDK2/build/sdk_core/
 )
 
set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ikd_Tree src/ikd_Tree.cpp)


add_library(lidar_slam SHARED
  src/lidar_slam.cpp 
  src/preprocess.cpp
  src/Scancontext.cpp
  src/IMU_Processing.cpp
  src/localization.cpp
  src/backend.cpp

  ThirdParty/livox_ros_driver2/src/driver_node.cpp
  ThirdParty/livox_ros_driver2/src/lds.cpp
  ThirdParty/livox_ros_driver2/src/lds_lidar.cpp
  ThirdParty/livox_ros_driver2/src/lddc.cpp

  ThirdParty/livox_ros_driver2/src/comm/comm.cpp
  ThirdParty/livox_ros_driver2/src/comm/ldq.cpp
  ThirdParty/livox_ros_driver2/src/comm/semaphore.cpp
  ThirdParty/livox_ros_driver2/src/comm/lidar_imu_data_queue.cpp
  ThirdParty/livox_ros_driver2/src/comm/cache_index.cpp
  ThirdParty/livox_ros_driver2/src/comm/pub_handler.cpp

  ThirdParty/livox_ros_driver2/src/parse_cfg_file/parse_cfg_file.cpp
  ThirdParty/livox_ros_driver2/src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  ThirdParty/livox_ros_driver2/src/call_back/lidar_common_callback.cpp
  ThirdParty/livox_ros_driver2/src/call_back/livox_lidar_callback.cpp
)
target_link_libraries(lidar_slam ${Sophus_LIBRARIES} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES} gtsam  
ikd_Tree libfast_gicp.so fmt::fmt yaml-cpp liblivox_lidar_sdk_shared.so) 

add_executable(lidar_slam_start src/lidar_slam_start.cpp)


target_link_libraries(lidar_slam_start 
    lidar_slam 
    ${Sophus_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${Pangolin_LIBRARIES} 
    gtsam  
    ikd_Tree 
    libfast_gicp.so 
    fmt::fmt 
    yaml-cpp 
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
    ${OpenCV_LIBS}
    pthread) 
