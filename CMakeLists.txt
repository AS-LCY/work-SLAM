cmake_minimum_required(VERSION 3.16)
project(lidar_slam)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE "Debug")


# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# C++17 标准设置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

# 调试模式设置
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -march=native") #

# CPU 架构优化
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_compile_definitions(ARM_ARCH)
else()
  # set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops")
  # add_compile_definitions(X86_ARCH)
   # 修改X86架构优化部分，添加AVX支持
   set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops -mavx")

   set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3 -g -march=native -mtune=native -funroll-loops -mavx")

   add_compile_definitions(X86_ARCH)
   
   # 显式检查AVX支持
   include(CheckCXXCompilerFlag)
   check_cxx_compiler_flag("-mavx" COMPILER_SUPPORTS_AVX)
   if(COMPILER_SUPPORTS_AVX)
     message(STATUS "AVX指令集支持已启用")
   else()
     message(WARNING "当前编译器不支持AVX指令集")
   endif()
endif()

# 多核处理设置
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)|(aarch64)|(AARCH64)")
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 6)
    add_compile_definitions(MP_EN MP_PROC_NUM=5)
  elseif(N GREATER 4)
    add_compile_definitions(MP_EN MP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_compile_definitions(MP_EN MP_PROC_NUM=2)
  else()
    add_compile_definitions(MP_PROC_NUM=1)
  endif()
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()

# OpenMP 支持
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################### ROS2 依赖管理 #####################
# 使用 ament_auto 简化构建
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ament_auto_find_build_dependencies(
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   std_msgs
#   tf2
#   tf2_ros
#   image_transport
#   pcl_conversions
#   flbot_msgs
# )
find_package(flbot_msgs REQUIRED)


find_package(logTracer REQUIRED)


# 第三方库依赖
find_package(Boost REQUIRED COMPONENTS
  system thread chrono serialization
  filesystem date_time regex timer
)
# find_package(PCL 1.15 REQUIRED COMPONENTS common io filters)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)   

# set(GTSAM_DIR /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake)
find_package(GTSAM REQUIRED)
find_package(fast_gicp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(pcl_conversions REQUIRED)

message("GTSAM_INCLUDE_DIR:===========" ${GTSAM_INCLUDE_DIR} )
message("GTSAM_LIBRARIES:==========="  ${GTSAM_LIBRARIES} )

message("PCL_LIBRARIES:==========="  ${PCL_LIBRARIES} )
message("PCL_INCLUDE_DIRS:==========="  ${PCL_INCLUDE_DIRS} )

# 包含目录
include_directories(
  src/
  src/include
  ${fairland_msgs_INCLUDE_DIRS}
    # ${FAST_GICP_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${yaml-cpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
  /usr/local/include
)

# 使用 ament_auto_add_library 添加库
ament_auto_add_library(lidar_preproc SHARED
  src/lidar/lidar_preproc_parent.cpp 
  src/lidar/livox/lidar_preproc_Mid360.cpp 
  src/lidar/robosense/lidar_preproc_Airy.cpp 
  src/lidar/vanjee/lidar_preproc_Vanjee722.cpp 
  src/lidar/hesai/lidar_preproc_JT16.cpp 
  src/lidar/lanhai/lidar_preproc_M300.cpp 
)

target_link_libraries(lidar_preproc
  ${PCL_LIBRARIES}
  Boost::boost
) 

# SLAM核心库
ament_auto_add_library(lidar_slam SHARED
  src/lidar_slam/common_lib.cpp 
  src/lidar_slam/lidar_slam.cpp 
  src/lidar_slam/localization.cpp
  src/lidar_slam/backend.cpp
  src/lidar_slam/IMU_Processing.cpp
  src/lidar_slam/ikd_Tree.cpp
  src/lidar_slam/Scancontext.cpp
  src/lidar_slam/cloud_map.cpp
  src/lidar_slam/global_localization.cpp
  src/lidar_slam/ekf_smoother.cpp
)

target_link_libraries(lidar_slam 
  ${Sophus_LIBRARIES}
  ${PCL_LIBRARIES}
  ${fmt_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
  ${GTSAM_LIBRARIES}
  lidar_preproc
  Boost::system
  Boost::thread
  Boost::chrono
  Boost::filesystem
  Boost::date_time
  Boost::regex
  Boost::timer
  Boost::serialization
  # /usr/local/lib/libfast_gicp.so
  gtsam
  ${logTracer_LIBRARY}
)

# SLAM主节点 - 使用 ament_auto_add_executable
ament_auto_add_executable(lidar_slam_node
  src/slam_main.cpp
  src/node/localization_module.cpp
  src/node/module_ctrl_callback.cpp 
  src/node/publish_common.cpp 
)

target_link_libraries(lidar_slam_node 
  lidar_slam
  lidar_preproc
  # flbot_msgs
)

# 通用数学库
# ament_auto_add_library(mango_common SHARED
#   src/fusion_node/common/numerical_process.cpp 
#   src/fusion_node/common/quaternion.cpp 
#   src/fusion_node/common/vector.cpp 
# )

# 融合定位节点
# ament_auto_add_executable(localization_fusion_node
#   src/fusion_node/fusion_main.cpp
#   src/fusion_node/localization_fusion_interface.cpp
#   src/fusion_node/ekf_fusion/ekf_localization_fusion.cpp 
#   src/fusion_node/ekf_fusion/pose_ekf.cpp 
#   src/slipping/detect_slipping.cpp
#   src/lidar_slam/common_lib.cpp 
# )

# target_link_libraries(localization_fusion_node 
#   mango_common
#   lidar_slam
# )


execute_process(
    COMMAND git log -1 --pretty=format:"Commit: %H%nAuthor: %an%nDate: %ad%nMessage: %s"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_INFO
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

file(WRITE ${CMAKE_BINARY_DIR}/version.txt "${GIT_INFO}\n")

install(
    FILES ${CMAKE_BINARY_DIR}/version.txt
    DESTINATION .
)

############# 安装配置 #############
# 安装其他资源文件
install(DIRECTORY 
  launch config rviz_cfg params
  DESTINATION share/${PROJECT_NAME}
)

# 使用 ament_auto_package 自动处理安装和导出
ament_auto_package(
  # # 显式添加非自动检测的依赖
  # DEPENDENCIES
  # Eigen3
  # PCL
  # Sophus
  # GTSAM
  # yaml-cpp
  # fmt
  # Boost
)