cmake_minimum_required(VERSION 3.0.2)
project(lidar_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


# SET(CMAKE_BUILD_TYPE "Debug")  
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
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

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



################### check CPU type #####################
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  # add_definitions(-DCPU_AARCH64)
  set(CPU_TYPE "aarch64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  # add_definitions(-DCPU_X86)
  set(CPU_TYPE "x86")
else()
  # add_definitions(-DCPU_UNKNOWN)
  set(CPU_TYPE "unknown")
  message("Unknown CPU type!")
endif()
message("********* CPU: ${CPU_TYPE} *********")
#---------------- check CPU type end --------------------

### setting ${THIRDPARTY_DIR}
set(THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/flbot_thirdparty)  
# set(THIRDPARTY_DIR /home/pan/catkin_ws/ws_localization2/src/flbot_thirdparty)  
message("********* THIRDPARTY_DIR: ${THIRDPARTY_DIR} *********")


################### find_package #####################
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  eigen_conversions
  fairland_msgs
)

find_package(Boost 1.54 REQUIRED COMPONENTS
  system
  thread
  chrono
)

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig)

################### set thirdparty #####################
### setting fast_gicp
set(FAST_GICP_INCLUDE_DIR ${THIRDPARTY_DIR}/fast_gicp/${CPU_TYPE}/include/)
set(FAST_GICP_LIBRARY_DIR ${THIRDPARTY_DIR}/fast_gicp/${CPU_TYPE}/lib/)
file(GLOB FAST_GICP_LIBRARY
    "${FAST_GICP_LIBRARY_DIR}/libfast_gicp.so"
)

### setting livox_sdk
set(LIVOX_SDK2_INCLUDE_DIR ${THIRDPARTY_DIR}/Livox-SDK2/${CPU_TYPE}/include)
set(LIVOX_SDK2_LIBRARY_DIR ${THIRDPARTY_DIR}/Livox-SDK2/${CPU_TYPE}/lib)

find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    ${THIRDPARTY_DIR}/Livox-SDK2/x86/lib)

### setting Eigen3
set(Eigen3_DIR ${THIRDPARTY_DIR}/eigen3/${CPU_TYPE}/share/eigen3/cmake)
find_package(Eigen3 REQUIRED PATHS ${Eigen3_DIR} NO_DEFAULT_PATH)
message("********* Eigen3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR} *********")

### setting fmt
set(fmt_DIR ${THIRDPARTY_DIR}/fmt/${CPU_TYPE}/lib/cmake/fmt)
find_package(fmt REQUIRED PATHS ${fmt_DIR} NO_DEFAULT_PATH )
# set(fmt_INCLUDE_DIRS ${THIRDPARTY_DIR}/fmt/${CPU_TYPE}/)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(fmt REQUIRED IMPORTED_TARGET fmt)
if(fmt_FOUND)
  message(STATUS "Found fmt, fmt_INCLUDE_DIRS: ${fmt_INCLUDE_DIRS}")
  # 其他使用 fmt_INCLUDE_DIRS 的命令
else()
  message(FATAL_ERROR "fmt not found")
endif()

message("********* fmt_FOUND: ${fmt_FOUND} *********")
message("********* fmt_INCLUDE_DIR: ${fmt_INCLUDE_DIR} *********")
message("********* fmt_INCLUDE_DIRS: ${fmt_INCLUDE_DIRS} *********")
message("********* fmt_LIBRARY: ${fmt_LIBRARY} *********")
message("********* fmt_LIBRARIES: ${fmt_LIBRARIES} *********")
message("********* fmt_VERSION: ${fmt_VERSION} *********")
message("********* fmt_ROOT_DIR: ${fmt_ROOT_DIR} *********")
message("********* fmt_ROOT: ${fmt_ROOT} *********")


# file(GLOB FMT_LIBRARIES
#     "${fmt_DIR}/lib/libfmt.so*"
# )
# # find_package( fmt REQUIRED )
# set(FMT_INCLUDE_DIRS ${fmt_DIR}../../../include)
# # find_package(fmt REQUIRED)
# message("********* FMT_LIBRARIES: ${FMT_LIBRARIES} *********")
# # find_package(fmt REQUIRED PATHS_ONLY PATHS ${fmt_DIR} NO_DEFAULT_PATH)

### setting Sophus
set(Sophus_DIR ${THIRDPARTY_DIR}/Sophus/${CPU_TYPE}/share/sophus/cmake)
find_package(Sophus REQUIRED)
set(Sophus_INCLUDE_DIRS ${THIRDPARTY_DIR}/Sophus/${CPU_TYPE}/include/)
message("********* Sophus_INCLUDE_DIRS: ${Sophus_INCLUDE_DIRS} *********")
# message("********* Sophus_LIBRARIES: ${Sophus_LIBRARIES} *********")
# Sophus 安装的库为模板类，没有libs


### setting Pangolin
set (Pangolin_DIR ${THIRDPARTY_DIR}/Pangolin/${CPU_TYPE}/lib/cmake/Pangolin)
find_package(Pangolin REQUIRED)
# set (Pangolin_INCLUDE_DIRS ${THIRDPARTY_DIR}/Pangolin/${CPU_TYPE}/include)
set (Pangolin_LIBRARY_DIRS ${THIRDPARTY_DIR}/Pangolin/${CPU_TYPE}/lib)
# file(GLOB Pangolin_LIBRARIES
#     "${Pangolin_LIBRARY_DIRS}/lib*"
# )

message("********* Pangolin_INCLUDE_DIRS: ${Pangolin_INCLUDE_DIRS} *********")
message("********* Pangolin_LIBRARIES: ${Pangolin_LIBRARIES} *********")

### setting OpenCV
set (OpenCV_DIR ${THIRDPARTY_DIR}/opencv/${CPU_TYPE}/opencv-4.5.4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED QUIET)
set (OpenCV_LIBRARY_DIRS ${THIRDPARTY_DIR}/opencv/${CPU_TYPE}/opencv-4.5.4/lib/)
message("********* OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS} *********")
message("********* OpenCV_LIBRARIES: ${OpenCV_LIBRARIES} *********")

### setting gtsam
set (GTSAM_DIR ${THIRDPARTY_DIR}/gtsam/${CPU_TYPE}/lib/cmake/GTSAM)
find_package(GTSAM PATHS ${GTSAM_DIR} NO_DEFAULT_PATH REQUIRED QUIET)
message("********* GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR} *********")
message("********* gtsam: ${gtsam} *********")

set (GTSAMCMakeTools_DIR ${THIRDPARTY_DIR}/gtsam/${CPU_TYPE}/lib/cmake/GTSAMCMakeTools)
find_package(GTSAMCMakeTools)

message("********* PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS} *********")
message("********* find_package end *********")

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge 
  geometry_msgs 
  image_transport 
  nav_msgs 
  pcl_ros 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs 
  tf
  fairland_msgs

  DEPENDS 
  # EIGEN3 
  PCL  
  # GTSAM 
  INCLUDE_DIRS
)


set(flbot_msgs_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../devel/include)

###########
## Build ##
###########

include_directories(
  src/include
  src/include/lidar_slam
  src/include/node
  src/include/livox_datatype
  ${flbot_msgs_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${FAST_GICP_INCLUDE_DIR}
  ${LIVOX_SDK2_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  # below TODO
  ${YAML_CPP_INCLUDE_DIR}
  ${APR_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
  )


# link directories
link_directories(
  ${FAST_GICP_LIBRARY_DIR}
  ${LIVOX_SDK2_LIBRARY_DIR}
  ${OpenCV_LIBRARY_DIRS}
  ${Pangolin_LIBRARY_DIRS}
)
 
set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ikd_Tree src/lidar_slam/ikd_Tree.cpp)
# add_library(livox_sdk SHARED IMPORTED)
# set_target_properties(livox_sdk PROPERTIES IMPORTED_LOCATION ${LIVOX_SDK2_LIBRARY_DIR}/liblivox_lidar_sdk_shared.so)

add_library(Scancontext src/lidar_slam/Scancontext.cpp)
target_link_libraries(Scancontext ${PCL_LIBRARIES} ${OpenCV_LIBS}) 

add_library(sensor_data_preproc 
  src/lidar_slam/preprocess.cpp 
  src/lidar_slam/IMU_Processing.cpp
  )
target_link_libraries(sensor_data_preproc ${PCL_LIBRARIES} ${OpenCV_LIBS}) 

# build: slam core
add_library(lidar_slam SHARED
  src/lidar_slam/lidar_slam.cpp 
  src/lidar_slam/localization.cpp
  src/lidar_slam/backend.cpp
  src/lidar_slam/cloud_map.cpp
  # src/lidar_slam/preprocess.cpp
  # src/lidar_slam/IMU_Processing.cpp
)
target_link_libraries(lidar_slam 
  fmt::fmt 
  ${Sophus_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${Pangolin_LIBRARIES} 
  ${FAST_GICP_LIBRARY}
  ${OpenCV_LIBS}
  ${fmt_LIBRARIES}
  gtsam  
  yaml-cpp 
  ikd_Tree 
  Scancontext
  sensor_data_preproc
) 


# build: ros-node
add_executable(lidar_slam_node
  src/slam_main.cpp
  src/node/localization_module.cpp
  src/node/publish_common.cpp 
)


# add_dependencies(lidar_slam_node )

target_link_libraries(lidar_slam_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${Pangolin_LIBRARIES} 
  ${FAST_GICP_LIBRARY}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  lidar_slam
  # gtsam
  # fmt::fmt 
  yaml-cpp 
  ) 



#############
## Install ##
#############


file(GLOB OPENCV_LIBRARY "${THIRDPARTY_DIR}/opencv/${CPU_TYPE}/opencv-4.5.4/lib/libopencv*")

install(TARGETS lidar_slam_node lidar_slam ikd_Tree Scancontext sensor_data_preproc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(FILES 
  ${FAST_GICP_LIBRARY}
  ${cv_bridge_LIBRARIES}
  #  ${OPENCV_LIBRARY} 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATITestON}
# )


