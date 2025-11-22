#!/bin/bash
# ARM交叉编译环境配置文件
# 所有可配置参数集中在此，其他脚本无需修改

# --------------------------
# 基础设备配置
# --------------------------
ARM_IP="172.18.0.16"       # arm设备ip
ARM_USER="root"             # 登录用户名
ARM_PASS="root"             # 登录密码
SYSROOT_DIR="$HOME/arm-rootfs"  # 本地交叉编译根目录
TOOLCHAIN_FILE="$(dirname "$0")/aarch64_toolchain.cmake"  # 生成的工具链文件路径

# --------------------------
# 项目目录配置
# --------------------------
PROJECT_DIR=$(dirname "$(pwd)")  # 项目根目录（默认为脚本所在目录的父目录）
ARM_BUILD_DIR="${PROJECT_DIR}/build_arm"      # ARM编译目录
ARM_INSTALL_DIR="${PROJECT_DIR}/install_arm"  # ARM安装目录
ARM_CCACHE_DIR="${PROJECT_DIR}/.ccache_arm"   # ARM专用ccache目录

# --------------------------
# 基础依赖路径配置
# --------------------------
PYTHON_LIB_ROOTFS="$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/libpython3.10.so"
PYTHON_LIB_SYMLINK="/usr/lib/aarch64-linux-gnu/libpython3.10.so"
CMAKE_C_COMPILER="/usr/bin/aarch64-linux-gnu-gcc"
CMAKE_CXX_COMPILER="/usr/bin/aarch64-linux-gnu-g++"

# --------------------------
# 系统目录配置 (用于同步基础系统文件)
# --------------------------
SYS_INCLUDE_DIRS=(
    "/usr/include/stdlib.h"
    "/usr/include/aarch64-linux-gnu/"
    "/usr/include/"
)
SYS_LIB_DIRS=(
    "/lib/aarch64-linux-gnu/"
    "/usr/lib/aarch64-linux-gnu/"
    "/usr/lib/gcc/aarch64-linux-gnu/11/"
    "/usr/local/lib/"
)
SYS_LINK_FIXES=(
    # 格式: 链接路径:目标路径
    "$SYSROOT_DIR/lib/aarch64-linux-gnu/libdl.so:libdl.so.2"
    "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/libpthread.so:libpthread.so.0"
    "$SYSROOT_DIR/lib/ld-linux-aarch64.so.1:$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1"
)

# --------------------------
# 项目通用依赖 (Python、ROS2等)
# --------------------------
PROJECT_DEPS=(
    # 格式: 源路径:目标路径
    "/usr/include/python3.10:$SYSROOT_DIR/usr/include/"
    "/usr/lib/aarch64-linux-gnu/libpython3.10.so*:$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/"
    "/opt/ros/humble:$SYSROOT_DIR/opt/ros/"
    "/usr/include/spdlog:$SYSROOT_DIR/usr/include/"
    "/usr/include/fmt:$SYSROOT_DIR/usr/include/"
    "/usr/include/mqtt:$SYSROOT_DIR/usr/include/"
    "/usr/bin:$SYSROOT_DIR/usr/"
)

# --------------------------
# 第三方库配置 - 便于扩展
# --------------------------
# 每个库使用数组配置，包含:名称、头文件路径、库路径、CMake路径等
# 1. logTracer配置
LOGTRACER=(
    "NAME:logTracer"
    "INCLUDE_PATH:/usr/local/include/logTracer"
    "LIB_PATH:/usr/local/lib"
    "CMAKE_PATH:/usr/local/lib/cmake/logTracer"
    "DEST_INCLUDE:$SYSROOT_DIR/usr/local/include/"
    "DEST_LIB:$SYSROOT_DIR/usr/local/lib/"
    "DEST_CMAKE:$SYSROOT_DIR/usr/local/lib/cmake/logTracer/"
    "LIB_FILES:liblogTracer.so*"  # 可选，指定要同步的库文件
)

# 2. nlohmann_json配置
NLOHMANN_JSON=(
    "NAME:nlohmann_json"
    "INCLUDE_PATH:/usr/include/nlohmann"
    "LIB_PATH:"  # 无单独库路径
    "CMAKE_PATH:/usr/lib/cmake/nlohmann_json"
    "DEST_INCLUDE:$SYSROOT_DIR/usr/include/"
    "DEST_LIB:"  # 无单独库路径
    "DEST_CMAKE:$SYSROOT_DIR/usr/lib/cmake/nlohmann_json/"
)

# 3. Eigen3配置
EIGEN3=(
    "NAME:Eigen3"
    "INCLUDE_PATH:/usr/include/eigen3"
    "LIB_PATH:"  # 头文件库，无单独库路径
    "CMAKE_PATH:/usr/share/eigen3/cmake"
    "DEST_INCLUDE:$SYSROOT_DIR/usr/include/"
    "DEST_LIB:"  # 头文件库，无单独库路径
    "DEST_CMAKE:$SYSROOT_DIR/usr/share/eigen3/cmake/"
)

# 4. GDAL配置
GDAL=(
    "NAME:GDAL" # 库名称（用于日志输出）
    "INCLUDE_PATH:/usr/include/gdal" # ARM设备上的头文件路径
    "LIB_PATH:/usr/lib" # ARM设备上的库文件路径
    "CMAKE_PATH:"  # 无CMake配置文件（留空）
    "DEST_INCLUDE:$SYSROOT_DIR/usr/include/gdal/" # 本地头文件存放路径
    "DEST_LIB:$SYSROOT_DIR/usr/lib/"  # 本地库文件存放路径
    "DEST_CMAKE:"  # 无CMake路径       # 无CMake配置文件（留空）
)
# 这部分配置会被sync_third_party_lib函数解析，自动同步头文件和库文件
# 同步逻辑：INCLUDE_PATH下的所有文件 → DEST_INCLUDE，LIB_PATH下的libgdal.so* → DEST_LIB
# GDAL 有额外的依赖库（如libarmadillo、libogdi等），这些库本身也是独立的第三方库，但因为它们仅作为 GDAL 的依赖存在，所以通过数组单独列出并同步：
# GDAL依赖库列表
GDAL_DEP_LIBS=("libarmadillo.so" "libarmadillo.so.10" "libarmadillo.so.10.8.2" 
               "libmfhdfalt.so" "libmfhdfalt.so.0" "libmfhdfalt.so.0.0.0"
               "libdfalt.so" "libdfalt.so.0" "libdfalt.so.0.0.0"
               "libogdi.so" "libogdi.so.4" "libogdi.so.4.1")

# GDAL 有额外的依赖库（如libarmadillo、libogdi等），这些库本身也是独立的第三方库，但因为它们仅作为 GDAL 的依赖存在，所以通过数组单独列出并同步
# 同步逻辑（在sync_all_deps中）
# for lib in "${GDAL_DEP_LIBS[@]}"; do
#     sshpass -p "$ARM_PASS" rsync -av \
#         "$ARM_USER@$ARM_IP:/usr/lib/$lib" "$SYSROOT_DIR/usr/lib/$lib"
# done
# 5. BLAS/LAPACK配置
# BLAS_LAPACK=(
#     "NAME:BLAS/LAPACK"
#     "INCLUDE_PATH:"  # 无单独头文件路径
#     "LIB_PATH:/usr/lib/aarch64-linux-gnu"
#     "CMAKE_PATH:"  # 无CMake路径
#     "DEST_INCLUDE:"  # 无单独头文件路径
#     "DEST_LIB:$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/"
#     "DEST_CMAKE:"  # 无CMake路径
# )
# 6. gtsam配置
GTSAM=(
    "NAME:gtsam"
    "INCLUDE_PATH:/usr/local/include/gtsam"
    "LIB_PATH:/usr/local/lib"
    "CMAKE_PATH:/usr/local/lib/cmake"
    "DEST_INCLUDE:$SYSROOT_DIR/usr/local/include/"
    "DEST_LIB:$SYSROOT_DIR/usr/local/lib/"
    "DEST_CMAKE:$SYSROOT_DIR/usr/local/lib/cmake/"
    "LIB_FILES:libmetis-gtsam.so*"  # 可选，指定要同步的库文件
)

# 7. sophus配置
SOPHUS=(
    "NAME:sophus"
    "INCLUDE_PATH:/usr/local/include/sophus"
    "CMAKE_PATH:/usr/local/share/sophus/cmake"
    "DEST_INCLUDE:$SYSROOT_DIR/usr/local/include/"
    "DEST_CMAKE:$SYSROOT_DIR/usr/local/share/sophus/cmake/"
)


# 线性代数的依赖库，不需要头文件，只需要库文件
# BLAS/LAPACK文件列表
BLAS_LAPACK_FILES=(
    "/usr/lib/aarch64-linux-gnu/libblas.so"
    "/usr/lib/aarch64-linux-gnu/libblas.so.3"
    "/usr/lib/aarch64-linux-gnu/blas/libblas.so"
    "/usr/lib/aarch64-linux-gnu/blas/libblas.so.3"
    "/usr/lib/aarch64-linux-gnu/blas/libblas.so.3.10.0"
    "/usr/lib/aarch64-linux-gnu/liblapack.so"
    "/usr/lib/aarch64-linux-gnu/liblapack.so.3"
    "/usr/lib/aarch64-linux-gnu/lapack/liblapack.so"
    "/usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3"
    "/usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3.10.0"
)

# --------------------------
# 验证文件列表 - 用于同步后检查
# --------------------------
CRITICAL_FILES=(
    # 系统基础文件
    "$SYSROOT_DIR/usr/include/stdlib.h"
    "$SYSROOT_DIR/usr/include/features.h"
    "$SYSROOT_DIR/usr/include/aarch64-linux-gnu/c++/11/bits/c++config.h"
    "$SYSROOT_DIR/lib/ld-linux-aarch64.so.1"
    "$SYSROOT_DIR/lib/aarch64-linux-gnu/libc.so.6"
    "$TOOLCHAIN_FILE"  # 工具链文件
    liblapack.so.3.10.0
    # 项目依赖
    "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/libpython3.10.so"
    "$SYSROOT_DIR/opt/ros/humble/include/rclcpp/rclcpp/rclcpp.hpp"
    "$SYSROOT_DIR/usr/include/spdlog/spdlog.h"
    "$SYSROOT_DIR/usr/include/fmt/core.h"
    
    # 第三方库
    "$SYSROOT_DIR/usr/local/include/logTracer/tracer.h"
    "$SYSROOT_DIR/usr/local/lib/liblogTracer.so"
    "$SYSROOT_DIR/usr/local/lib/cmake/logTracer/logTracerConfig.cmake"
    "$SYSROOT_DIR/usr/include/nlohmann/json.hpp"
    "$SYSROOT_DIR/usr/lib/cmake/nlohmann_json/nlohmann_jsonConfig.cmake"
    "$SYSROOT_DIR/usr/include/eigen3/Eigen/Eigen"
    "$SYSROOT_DIR/usr/share/eigen3/cmake/Eigen3Config.cmake"
    "$SYSROOT_DIR/usr/include/gdal/gdal.h" # 验证头文件是否同步成功
    "$SYSROOT_DIR/usr/lib/libgdal.so.30" # 验证核心库文件是否同步成功
    "$SYSROOT_DIR/usr/lib/libarmadillo.so.10"
    "$SYSROOT_DIR/usr/lib/libmfhdfalt.so.0"
    "$SYSROOT_DIR/usr/lib/libdfalt.so.0"
    "$SYSROOT_DIR/usr/lib/libogdi.so.4.1"
    "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/libblas.so.3"
    "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/blas/libblas.so.3.10.0"
    "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/liblapack.so.3"
    "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3.10.0"
    "$SYSROOT_DIR/usr/local/lib/libgtsam.so.4.1.1"
)
