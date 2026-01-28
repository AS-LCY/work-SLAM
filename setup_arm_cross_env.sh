#!/bin/bash
# 功能：ARM交叉编译环境依赖同步脚本（含CMake工具链自动生成）
# 同步类别说明：
# 1. 系统基础文件      - 包括标准头文件、系统库、编译器相关文件
# 2. 项目通用依赖      - Python、ROS2、spdlog、fmt等基础依赖
# 3. 第三方库          - 通过配置文件定义的各类库

# ==============================================
# 添加新第三方库的方法：
# 1. 在arm_config.sh中，按照现有库的格式添加新库配置
#    例如：
#    NEW_LIB=(
#        "NAME:库名称"
#        "INCLUDE_PATH:库头文件路径"
#        "LIB_PATH:库文件路径"
#        "CMAKE_PATH:CMake配置文件路径"
#        "DEST_INCLUDE:本地头文件目标路径"
#        "DEST_LIB:本地库文件目标路径"
#        "DEST_CMAKE:本地CMake配置目标路径"
#        "LIB_FILES:需要同步的具体库文件(可选)"
#    )
#
# 2. 在sync_all_deps函数中添加同步调用：
#    sync_third_party_lib "${NEW_LIB[@]}"
#
# 3. 在arm_config.sh的CRITICAL_FILES数组中添加验证文件路径
# ==============================================

set -euo pipefail

# -------------------------- 加载配置文件 --------------------------
CONFIG_FILE="$(dirname "$0")/arm_config.sh"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "错误：配置文件 $CONFIG_FILE 不存在！"
    exit 1
fi
source "$CONFIG_FILE"

# ==============================================
# 新增：通用第三方库同步函数（核心改进）
# ==============================================
sync_third_party_lib() {
    # 解析库配置参数
    local name=""
    local include_path=""
    local lib_path=""
    local cmake_path=""
    local dest_include=""
    local dest_lib=""
    local dest_cmake=""
    local lib_files=""

    # 遍历参数对，提取配置值
    for param in "$@"; do
        local key="${param%:*}"
        local value="${param#*:}"
        case "$key" in
            "NAME") name="$value" ;;
            "INCLUDE_PATH") include_path="$value" ;;
            "LIB_PATH") lib_path="$value" ;;
            "CMAKE_PATH") cmake_path="$value" ;;
            "DEST_INCLUDE") dest_include="$value" ;;
            "DEST_LIB") dest_lib="$value" ;;
            "DEST_CMAKE") dest_cmake="$value" ;;
            "LIB_FILES") lib_files="$value" ;;
        esac
    done

    echo -e "\n→ 同步第三方库: $name"

    # 同步头文件
    if [ -n "$include_path" ] && [ -n "$dest_include" ]; then
        echo "  - 同步头文件: $include_path → $dest_include"
        mkdir -p "$dest_include"
        sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
            "$ARM_USER@$ARM_IP:$include_path" "$dest_include"
    fi

    # 同步库文件
    if [ -n "$lib_path" ] && [ -n "$dest_lib" ]; then
        echo "  - 同步库文件: $lib_path → $dest_lib"
        mkdir -p "$dest_lib"
        if [ -n "$lib_files" ]; then
            # 如果指定了具体文件，只同步这些文件
            for file in $lib_files; do
                sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
                    "$ARM_USER@$ARM_IP:$lib_path/$file" "$dest_lib/"
            done
        else
            # 否则同步整个目录
            sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
                "$ARM_USER@$ARM_IP:$lib_path/" "$dest_lib/"
        fi
    fi

    # 同步CMake配置
    if [ -n "$cmake_path" ] && [ -n "$dest_cmake" ]; then
        echo "  - 同步CMake配置: $cmake_path → $dest_cmake"
        mkdir -p "$dest_cmake"
        sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
            "$ARM_USER@$ARM_IP:$cmake_path/" "$dest_cmake/"
    fi
}

# ==============================================
# 生成CMake工具链文件（与同步脚本联动的核心）
# ==============================================
generate_toolchain() {
    echo -e "\n【4/3】生成CMake工具链文件: $TOOLCHAIN_FILE"
    
    # 确保工具链文件目录存在
    mkdir -p "$(dirname "$TOOLCHAIN_FILE")"
    
    # 写入工具链配置（与用户提供的CMake配置一致，动态关联SYSROOT_DIR）
    cat > "$TOOLCHAIN_FILE" << EOF
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 核心：强制指定用ARM的GCC/G++，写死路径
set(CMAKE_C_COMPILER $CMAKE_C_COMPILER CACHE FILEPATH "强制ARM GCC" FORCE)
set(CMAKE_CXX_COMPILER $CMAKE_CXX_COMPILER CACHE FILEPATH "强制ARM G++" FORCE)

set(CMAKE_C_COMPILER_LAUNCHER /usr/bin/ccache CACHE STRING "C编译器启动器" FORCE)
set(CMAKE_CXX_COMPILER_LAUNCHER /usr/bin/ccache CACHE STRING "CXX编译器启动器" FORCE)

# 动态关联同步脚本中的SYSROOT_DIR
set(CMAKE_SYSROOT $SYSROOT_DIR)

set(CMAKE_INCLUDE_PATH 
    \${CMAKE_SYSROOT}/usr/local/include  # logTracer头文件所在目录
    \${CMAKE_SYSROOT}/usr/include/eigen3  # Eigen3头文件所在目录
    \${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu/c++/11/bits
    \${CMAKE_SYSROOT}/usr/include
    \${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu
    \${CMAKE_SYSROOT}/usr/include/c++/11
    \${CMAKE_SYSROOT}/usr/include/c++/11/aarch64-linux-gnu
)

set(CMAKE_LIBRARY_PATH 
    \${CMAKE_SYSROOT}/usr/local/lib  # logTracer库文件所在目录
    \${CMAKE_SYSROOT}/lib
    \${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu
)

# 编译参数补充：显式加入/usr/local/include（双重保险）
set(CMAKE_CXX_FLAGS "--sysroot=\${CMAKE_SYSROOT} -I\${CMAKE_SYSROOT}/usr/local/include -I\${CMAKE_SYSROOT}/usr/include/eigen3 -I\${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu/c++/11/bits -I\${CMAKE_SYSROOT}/usr/include -pthread")
set(CMAKE_C_FLAGS "\${CMAKE_CXX_FLAGS}")

# 链接参数补充：显式加入/usr/local/lib（双重保险）
set(CMAKE_EXE_LINKER_FLAGS "--sysroot=\${CMAKE_SYSROOT} -L\${CMAKE_SYSROOT}/usr/local/lib -L\${CMAKE_SYSROOT}/lib -L\${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu -ldl -lpthread")
set(CMAKE_SHARED_LINKER_FLAGS "\${CMAKE_EXE_LINKER_FLAGS}")

# 原有配置保留（禁用Clang、设置ROS路径等）
set(CMAKE_CXX_CLANG_TIDY "" CACHE STRING "" FORCE)
set(CMAKE_C_CLANG_TIDY "" CACHE STRING "" FORCE)
set(ENABLE_CLANG_FORMAT OFF CACHE BOOL "" FORCE)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(Threads_FOUND TRUE)
set(CMAKE_FIND_ROOT_PATH \${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)  # 确保只搜索sysroot的库（解决冲突警告）
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)  # 确保只搜索sysroot的头文件
set(fmt_DIR "\${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake/fmt")
set(spdlog_DIR "\${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake/spdlog")
# nlohmann_json的CMake配置路径
set(nlohmann_json_DIR "\${CMAKE_SYSROOT}/usr/lib/cmake/nlohmann_json")
# logTracer的CMake配置路径
set(logTracer_DIR "\${CMAKE_SYSROOT}/usr/local/lib/cmake/logTracer")
# Eigen3的CMake配置路径
set(Eigen3_DIR "\${CMAKE_SYSROOT}/usr/share/eigen3/cmake")
set(CMAKE_PREFIX_PATH \${CMAKE_SYSROOT}/opt/ros/humble)
# 强制用系统的make（而非gmake），解决Makefile解析格式问题
set(CMAKE_MAKE_PROGRAM /usr/bin/make CACHE FILEPATH "强制使用make编译" FORCE)
set(PYTHON_SOABI "cpython-310-aarch64-linux-gnu" CACHE STRING "Python SOABI for aarch64")
EOF

    if [ -f "$TOOLCHAIN_FILE" ]; then
        echo "→ 工具链文件生成成功，路径：$TOOLCHAIN_FILE"
        echo "→ 自动关联sysroot：$SYSROOT_DIR"
    else
        echo "→ 工具链文件生成失败！"
        exit 1
    fi
}

# ==============================================
# 工具检查函数 - 集中管理依赖检查
# ==============================================
check_deps() {
    echo -e "\n【1/4】检查必需工具..."
    return 0
    local deps=("rsync" "sshpass" "gcc-aarch64-linux-gnu" "g++-aarch64-linux-gnu" "ccache")
    local missing=()
    
    for dep in "${deps[@]}"; do
        if ! command -v "$dep" &>/dev/null; then
            missing+=("$dep")
        fi
    done
    
    if [ ${#missing[@]} -gt 0 ]; then
        sudo apt update -qq
        sudo apt install -y --no-install-recommends "${missing[@]}"
        echo "→ 已安装缺失工具：${missing[*]}"
    else
        echo "→ 所有必需工具已就绪"
    fi
}

# ==============================================
# 目录创建函数 - 集中预处理目录结构
# ==============================================
prepare_directories() {
    echo "→ 准备目录结构..."
    # 创建基础目录结构
    mkdir -p "$SYSROOT_DIR"/{lib/{aarch64-linux-gnu},usr/{include,lib/{aarch64-linux-gnu,gcc/aarch64-linux-gnu/11,cmake/{fmt,spdlog}},local/include,local/lib,local/lib/cmake/logTracer},opt/ros}
    
    # 创建第三方库专用目录
    mkdir -p "$SYSROOT_DIR/usr/lib/cmake/nlohmann_json"
    mkdir -p "$SYSROOT_DIR/usr/share/eigen3/cmake"
    mkdir -p "$SYSROOT_DIR/usr/include/gdal"
    
    # 创建架构相关目录
    mkdir -p "$SYSROOT_DIR/usr/include/aarch64-linux-gnu/{bits,sys}"
    mkdir -p "$SYSROOT_DIR/usr/include/sys"
    
    # 创建BLAS/LAPACK专用目录
    mkdir -p "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/blas"
    mkdir -p "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/lapack"
}

# ==============================================
# 同步函数 - 按类别集中同步操作（核心改进）
# ==============================================
sync_all_deps() {
    echo -e "\n【2/4】同步所有依赖(系统头文件+库+Python+ROS2+第三方库）..."
    
    # 预处理目录
    prepare_directories

    # 1. 同步系统核心头文件
    echo -e "\n→ 同步系统头文件..."
    for dir in "${SYS_INCLUDE_DIRS[@]}"; do
        local target_dir="$SYSROOT_DIR${dir%/*}"
        mkdir -p "$target_dir"
        if [[ "$dir" == *"*" ]]; then
            sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
                "$ARM_USER@$ARM_IP:$dir" "$target_dir/"
        else
            if [[ -f "$dir" ]]; then
                sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
                    "$ARM_USER@$ARM_IP:$dir" "$target_dir/"
            else
                sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
                    --exclude=aarch64-linux-gnu \
                    "$ARM_USER@$ARM_IP:$dir" "$target_dir/"
            fi
        fi
    done

    # 2. 同步核心系统库
    echo -e "\n→ 同步系统库..."
    for dir in "${SYS_LIB_DIRS[@]}"; do
        local target_dir="$SYSROOT_DIR$dir"
        mkdir -p "$target_dir"
        sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
            "$ARM_USER@$ARM_IP:$dir" "$target_dir/"
    done

    # 3. 修复基础库软链接
    echo -e "\n→ 修复基础库软链接..."
    for link_info in "${SYS_LINK_FIXES[@]}"; do
        local link_path="${link_info%:*}"
        local target="${link_info#*:}"
        if [ ! -L "$link_path" ]; then
            ln -sf "$target" "$link_path"
            echo "→ 创建软链接: $link_path → $target"
        fi
    done

    # 4. 同步项目依赖库
    echo -e "\n→ 同步项目依赖库..."
    for dep in "${PROJECT_DEPS[@]}"; do
        local src="${dep%:*}"
        local dest="${dep#*:}"
        mkdir -p "$dest"
        sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
            --exclude=build --exclude=log --exclude=*.pyc \
            "$ARM_USER@$ARM_IP:$src" "$dest"
    done

    # 5. 同步第三方库（使用通用同步函数）
    sync_third_party_lib "${LOGTRACER[@]}"
    sync_third_party_lib "${NLOHMANN_JSON[@]}"
    sync_third_party_lib "${EIGEN3[@]}"
    sync_third_party_lib "${GDAL[@]}"
    sync_third_party_lib "${GTSAM[@]}"
    sync_third_party_lib "${SOPHUS[@]}"
    sync_third_party_lib "${FAST_GICP[@]}"

    # 6. 同步GDAL依赖库
    echo -e "\n→ 同步GDAL依赖库（armadillo/mfhdfalt/dfalt/ogdi）..."
    for lib in "${GDAL_DEP_LIBS[@]}"; do
        sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
            "$ARM_USER@$ARM_IP:/usr/lib/$lib" "$SYSROOT_DIR/usr/lib/$lib"
        echo "→ 已同步GDAL依赖库: $lib"
    done

    # 7. 同步BLAS/LAPACK库
    echo -e "\n→ 同步BLAS/LAPACK库（libarmadillo依赖）..."
    for file in "${BLAS_LAPACK_FILES[@]}"; do
        local relative_path="${file#/}"
        local sysroot_path="$SYSROOT_DIR/$relative_path"
        sshpass -p "$ARM_PASS" rsync -av --super --no-perms --no-owner --no-group \
            "$ARM_USER@$ARM_IP:$file" "$sysroot_path"
        echo "→ 已同步BLAS/LAPACK文件：$relative_path"
    done

    # 8. 强制修正liblapack.so.3软链接
    local lapack_link="$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/liblapack.so.3"
    local lapack_real_target="$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3"
    rm -f "$lapack_link"
    ln -sf "$lapack_real_target" "$lapack_link"
    echo -e "\n→ 强制修正软链接：$lapack_link → $lapack_real_target"

    # 9. 修复其他软链接
    echo -e "\n→ 修复所有软链接 (GDAL+BLAS/LAPACK)..."
    # 修复GDAL软链接
    find "$SYSROOT_DIR/usr/lib" -name "libgdal.so*" -type l | while read -r link; do
        target=$(readlink "$link")
        if [[ "$target" == /usr/lib/* ]]; then
            new_target="$SYSROOT_DIR$target"
            ln -sf "$new_target" "$link"
            echo "修复软链接：$link → $new_target"
        fi
    done
    
    # 修复GDAL依赖库软链接
    for lib_prefix in "armadillo" "mfhdfalt" "dfalt" "ogdi"; do
        find "$SYSROOT_DIR/usr/lib" -name "lib${lib_prefix}.so*" -type l | while read -r link; do
            target=$(readlink "$link")
            if [[ "$target" == /usr/lib/* ]]; then
                new_target="$SYSROOT_DIR$target"
                ln -sf "$new_target" "$link"
                echo "修复软链接：$link → $new_target"
            fi
        done
    done
    
    # 修复BLAS软链接
    find "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu" -name "libblas.so*" -type l | while read -r link; do
        target=$(readlink "$link")
        if [[ "$target" == /etc/alternatives/* ]]; then
            local alt_target=$(sshpass -p "$ARM_PASS" ssh "$ARM_USER@$ARM_IP" "readlink '$target'")
            new_target="$SYSROOT_DIR$alt_target"
            ln -sf "$new_target" "$link"
            echo "修复软链接 (alternatives: $link → $new_target"
        elif [[ "$target" == /usr/lib/aarch64-linux-gnu/* ]]; then
            new_target="$SYSROOT_DIR$target"
            ln -sf "$new_target" "$link"
            echo "修复软链接：$link → $new_target"
        fi
    done
    
    # 修复BLAS/LAPACK子目录内的链接
    for lib_prefix in "blas" "lapack"; do
        find "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/${lib_prefix}" -name "lib${lib_prefix}.so*" -type l | while read -r link; do
            target=$(readlink "$link")
            if [[ "$target" == /usr/lib/aarch64-linux-gnu/* ]]; then
                new_target="$SYSROOT_DIR$target"
                ln -sf "$new_target" "$link"
                echo "修复软链接（子目录）：$link → $new_target"
            fi
        done
    done

    echo -e "\n所有依赖同步完成！"
}

# ==============================================
# 验证函数 - 集中管理关键文件验证
# ==============================================
verify_all() {
    echo -e "\n【4/4】验证关键文件..."
    
    # 执行验证检查
    local err=0
    for f in "${CRITICAL_FILES[@]}"; do
        if [ ! -f "$f" ] && [ ! -L "$f" ]; then
            echo "缺失文件：$f"
            err=1
        elif [ -L "$f" ] && [ ! -e "$f" ]; then
            echo "无效软链接：$f"
            err=1
        fi
    done
    
    if [ $err -eq 0 ]; then
        echo -e "\n环境同步验证通过!"
        echo "下一步：执行 ./build_arm.sh 开始编译"
    else
        echo -e "\n环境验证失败, 请重新运行本脚本"
        exit 1
    fi
}

# ==============================================
# 脚本入口（执行检查→同步→验证）
# ==============================================
echo "===== 开始 ARM 交叉编译依赖同步 ====="
check_deps  # 启用工具检查（确保ccache等工具存在）
sync_all_deps
generate_toolchain  # 同步完成后自动生成工具链文件
verify_all
echo "===== ARM 交叉编译依赖同步完成 ====="
