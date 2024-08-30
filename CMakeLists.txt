cmake_minimum_required(VERSION 3.0.2)
project(lidar_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# ADD_COMPILE_OPTIONS(-std=c++14 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
# set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_compile_options(-fPIC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -std=c++17 -fexceptions")
#set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
#set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wl, -rpath=./:./lib/")
#set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wl -rpath=${CMAKE_SOURCE_DIR}/../install/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


#get_filename_component(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
#add_compile_definitions(CURRENT_DIR="${CURRENT_DIR}")

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



################### find_package #####################
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # cv_bridge
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

find_package(Boost  REQUIRED COMPONENTS
  system
  thread
  chrono
  serialization
  filesystem
  date_time
  regex
  timer
  date_time
)

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig)
find_package(FMT REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# message("======= PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS} *********")
# message("======= PCL_LIBRARIES: ${PCL_LIBRARIES} *********")

catkin_package(
  CATKIN_DEPENDS 
  # cv_bridge 
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
  # ${FAST_GICP_INCLUDE_DIR}
  ${LIVOX_SDK2_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  #${Pangolin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  # below TODO
  ${YAML_CPP_INCLUDE_DIR}
  ${APR_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/local/include
  )


# link directories
link_directories(
  ${FAST_GICP_LIBRARY_DIR}
  ${LIVOX_SDK2_LIBRARY_DIR}
  # ${OpenCV_LIBRARY_DIRS}
  #${Pangolin_LIBRARY_DIRS}
)
 
set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})



# build: slam core
add_library(lidar_slam SHARED
  src/lidar_slam/lidar_slam.cpp 
  src/lidar_slam/localization.cpp
  src/lidar_slam/backend.cpp
  src/lidar_slam/preprocess.cpp
  src/lidar_slam/IMU_Processing.cpp
  src/lidar_slam/ikd_Tree.cpp
  src/lidar_slam/Scancontext.cpp
  src/lidar_slam/cloud_map.cpp
  src/lidar_slam/global_localization.cpp
)
target_link_libraries(lidar_slam 
  ${Sophus_LIBRARIES} 
  ${PCL_LIBRARIES} 
  #${Pangolin_LIBRARIES} 
  ${FAST_GICP_LIBRARY}
  ${OpenCV_LIBS}
  ${fmt_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam  
  fmt::fmt 
  yaml-cpp 
  Boost::system
  Boost::thread
  Boost::chrono
  Boost::filesystem
  Boost::date_time
  Boost::regex
  Boost::timer
  Boost::date_time
  Boost::serialization
  /usr/local/lib/libfast_gicp.so
#   ikd_Tree 
#   Scancontext
#   backend
#   sensor_data_preproc
) 

  
  
  

# build: ros-node
add_executable(lidar_slam_node
  src/slam_main.cpp
  src/node/localization_module.cpp
  src/node/module_ctrl_callback.cpp 
  src/node/publish_common.cpp 
  src/node/load_param.cpp 
)

# add_dependencies(lidar_slam_node )

target_link_libraries(lidar_slam_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  # ${Pangolin_LIBRARIES} 
  # ${FAST_GICP_LIBRARY}
  # ${OpenCV_LIBS}
  # ${cv_bridge_LIBRARIES}
  lidar_slam
  # gtsam
  # fmt::fmt 
  # yaml-cpp 
  ) 



#############
## Install ##
#############


# file(GLOB OPENCV_LIBS "${THIRDPARTY_DIR}/opencv/${CPU_TYPE}/lib/libopencv*")
file(GLOB PCL_LIBS "${THIRDPARTY_DIR}/pcl/${CPU_TYPE}/pcl-1.10.0/lib/libpcl*")
file(GLOB GTSAM_LIBS "${THIRDPARTY_DIR}/gtsam/${CPU_TYPE}/lib/lib*")
file(GLOB fmt_LIBS "${THIRDPARTY_DIR}/fmt/${CPU_TYPE}/lib/lib*")
#file(GLOB Pangolin_LIBS "${THIRDPARTY_DIR}/Pangolin/${CPU_TYPE}/lib/lib*")
file(GLOB YAML_CPP_LIBS "${THIRDPARTY_DIR}/yaml-cpp/${CPU_TYPE}/lib/lib*")
file(GLOB fast_gicp_LIBS "${THIRDPARTY_DIR}/fast_gicp/${CPU_TYPE}/lib/lib*")

install(TARGETS lidar_slam_node lidar_slam #ikd_Tree Scancontext backend sensor_data_preproc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(FILES 
  ${cv_bridge_LIBRARIES}
  # ${OPENCV_LIBS} 
  ${PCL_LIBS} 
  ${GTSAM_LIBS} 
  ${fmt_LIBS} 
  #${Pangolin_LIBS} 
  ${YAML_CPP_LIBS} 
  ${fast_gicp_LIBS} 
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


