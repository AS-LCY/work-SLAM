cmake_minimum_required(VERSION 3.5)
project(bbs3d)

set(CMAKE_BUILD_TYPE "Release")


find_package(Eigen3 REQUIRED)


## find OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


## find CUDA
option(BUILD_CUDA "Build GPU ver" OFF)
if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()


include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Use EIGEN3_INCLUDE_DIR")

include_directories(include)

add_library(cpu_bbs3d
  SHARED
  src/cpu_bbs3d/bbs3d.cpp
  src/cpu_bbs3d/voxelmaps.cpp
  src/cpu_bbs3d/voxelmaps_io.cpp
)

# colcon build --symlink-install build error
# install(TARGETS cpu_bbs3d LIBRARY DESTINATION lib)
# install(DIRECTORY include/ DESTINATION include)

install(TARGETS cpu_bbs3d
    EXPORT cpu_bbs3d-export
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT cpu_bbs3d-export
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cpu_bbs3d
    # NAMESPACE cpu_bbs3d::
    FILE cpu_bbs3dTargets.cmake
)

install(DIRECTORY include/cpu_bbs3d DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cpu_bbs3d)
install(DIRECTORY include/discrete_transformation DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cpu_bbs3d)
install(DIRECTORY include/pointcloud_iof DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cpu_bbs3d)

export(TARGETS ${MYPROJECT_EXPORTED_TARGETS} FILE cpu_bbs3d-exports.cmake)


if(BUILD_CUDA)
  cuda_add_library(gpu_bbs3d
    SHARED 
    src/gpu_bbs3d/bbs3d.cu
    src/gpu_bbs3d/calc_score.cu
    src/gpu_bbs3d/voxelmaps.cu
    src/gpu_bbs3d/stream_manager/check_error.cu
    src/gpu_bbs3d/voxelmaps_io.cu
  )

  install(TARGETS gpu_bbs3d DESTINATION lib)
endif()