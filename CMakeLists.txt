cmake_minimum_required(VERSION 2.8.3)
project(lidar_slam)

SET(CMAKE_BUILD_TYPE "Release")  

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

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

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver2
  message_generation
  eigen_conversions
)

find_package( GTSAMCMakeTools )
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package( FMT REQUIRED )
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  include
  include/serial_lib/include/
  )



catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs 
  DEPENDS EIGEN3 PCL  GTSAM 
  INCLUDE_DIRS
)

###########
## Build ##
###########
add_library(lidar_slam SHARED IMPORTED)
set_target_properties(lidar_slam PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/build/liblidar_slam.so )
add_library(gicp SHARED IMPORTED)
set_target_properties(gicp PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/ThirdParty/fast_gicp/build/devel/lib/libfast_gicp.so)
add_library(livox_sdk SHARED IMPORTED)
set_target_properties(livox_sdk PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/ThirdParty/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/ThirdParty/fast_gicp/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/ThirdParty/Livox-SDK2/include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/ThirdParty/livox_ros_driver2/3rdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/ThirdParty/livox_ros_driver2/src
)




message(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(slam_interface src/slam_interface.cpp)
target_link_libraries(slam_interface ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES} gicp lidar_slam ${OpenCV_LIBS} ${cv_bridge_LIBRARIES} gtsam fmt yaml-cpp  livox_sdk) 


#add_executable(camera_driver src/camera_driver.cpp include/serial_lib/src/serial.cc include/serial_lib/src/impl/unix.cc include/serial_lib/src/impl/list_ports/list_ports_linux.cc)
#target_link_libraries(camera_driver ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES})


#############
## Install ##
#############
install(TARGETS slam_interface 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/build/liblidar_slam.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/ThirdParty/fast_gicp/build/devel/lib/libfast_gicp.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/ThirdParty/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} 
        FILES_MATCHING PATTERN "*.h" 
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY lib/config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

