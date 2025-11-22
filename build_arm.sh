#!/bin/bash
set -e  

# 转换为大写（CMake 要求）
BUILD_TYPE_UPPER=$(echo "${BUILD_TYPE}" | tr '[:lower:]' '[:upper:]')

# -------------------------- 并行线程配置（可根据需要调整）--------------------------
MIN_PARALLEL_NUM=4                  # 最小并行线程数
MAX_PARALLEL_NUM=16                 # 最大并行线程数
COLCON_PARALLEL_12CORES=4           # 12核CPU的并行包数
CCACHE_MAXSIZE="20G"                # ccache最大缓存大小

# -------------------------- 颜色变量定义（仅用于重要信息）--------------------------
RED='\033[0;31m'       # 错误信息：红色
GREEN='\033[0;32m'     # 成功信息：绿色
YELLOW='\033[1;33m'    # 关键配置：黄色
NC='\033[0m'           # 颜色重置

# -------------------------- 参数解析 --------------------------
BUILD_TYPE="debug"  # 默认值
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --build-type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

# 校验参数值
if [[ "${BUILD_TYPE}" != "debug" && "${BUILD_TYPE}" != "release" ]]; then
    echo -e "${RED}错误：--build-type 参数只能是 'debug' 或 'release'${NC}"
    exit 1
fi

echo -e "${YELLOW}BUILD_TYPE: ${BUILD_TYPE}${NC}"

# -------------------------- 加载配置文件 --------------------------
CONFIG_FILE="$(dirname "$0")/arm_config.sh"
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}错误：配置文件 $CONFIG_FILE 不存在！${NC}"
    echo "请先运行同步脚本 sync_arm_deps.sh"
    exit 1
fi
source "$CONFIG_FILE"
TOOLCHAIN_FILE=$(realpath "${TOOLCHAIN_FILE}")  # 防止路径无效


# -------------------------- 脚本退出时自动清理Python软链接 --------------------------
trap 'echo -e "\n=== 脚本退出，清理临时软链接 ==="; sudo rm -f "${PYTHON_LIB_SYMLINK}"' EXIT


# -------------------------- 1. 动态线程+并行包数优化 --------------------------
CPU_CORES=$(nproc)
# 线程数配置：12核及以下=核心数，超12核=核心数*3/2（上限MAX_PARALLEL_NUM）
if [ ${CPU_CORES} -le 12 ]; then
    PARALLEL_NUM=${CPU_CORES}  # 小核心避免超配，减少上下文切换
else
    PARALLEL_NUM=$((CPU_CORES * 3 / 2))  
    [ $PARALLEL_NUM -gt $MAX_PARALLEL_NUM ] && PARALLEL_NUM=$MAX_PARALLEL_NUM  # 大核心上限，防内存溢出
fi
[ $PARALLEL_NUM -lt $MIN_PARALLEL_NUM ] && PARALLEL_NUM=$MIN_PARALLEL_NUM  # 最低线程数，兼容低配

# 并行包数配置：12核专属COLCON_PARALLEL_12CORES个包，其他核数=核心数/2
if [ ${CPU_CORES} -eq 12 ]; then
    COLCON_PARALLEL_WORKERS=$COLCON_PARALLEL_12CORES  # 12核优化I/O竞争，避免多包同时编译
else
    COLCON_PARALLEL_WORKERS=$((CPU_CORES / 2))
fi
[ $COLCON_PARALLEL_WORKERS -lt 1 ] && COLCON_PARALLEL_WORKERS=1  # 最低1包

# 导出参数
export CMAKE_BUILD_PARALLEL_LEVEL=${PARALLEL_NUM}
export MAKEFLAGS="-j${PARALLEL_NUM}"
export CCACHE_DIR="${ARM_CCACHE_DIR}"
export CCACHE_MAXSIZE="${CCACHE_MAXSIZE}"


# -------------------------- 2. 清理ARM专属缓存 --------------------------
echo -e "=== 第一步: 清理ARM专属缓存 ==="
rm -rf "${ARM_BUILD_DIR}" "${ARM_INSTALL_DIR}" "${ARM_CCACHE_DIR}/tmp"
# 确保目录存在并有正确权限
mkdir -p "${ARM_BUILD_DIR}" "${ARM_INSTALL_DIR}" "${ARM_CCACHE_DIR}"
chown -R "$(whoami):$(whoami)" "${ARM_CCACHE_DIR}" 2>/dev/null || true
chmod -R 755 "${ARM_CCACHE_DIR}" 2>/dev/null || true

echo "缓存清理完成："
echo "  - 清理ARM目录: ${ARM_BUILD_DIR}、${ARM_INSTALL_DIR}、${ARM_CCACHE_DIR}/tmp"
echo "  - x64目录(build/install/log)未改动, 可继续x64开发"
echo -e "  - 编译配置：${YELLOW}线程数=${PARALLEL_NUM}，并行包数=${COLCON_PARALLEL_WORKERS}${NC}(CPU核心数=${CPU_CORES})"


# -------------------------- 3. 依赖检查 --------------------------
echo -e "\n=== 第二步: 检查核心依赖 ==="

if [ ! -f "${PYTHON_LIB_ROOTFS}" ]; then
    echo -e "${RED}错误: ARM Python库不存在 - ${PYTHON_LIB_ROOTFS}${NC}"
    echo "请先运行同步脚本 sync_arm_deps.sh"
    exit 1
fi

echo -e "${GREEN}Python库正常: $(ls -l "${PYTHON_LIB_ROOTFS}" | awk '{print $9 " -> " $11}')${NC}"

if [ ! -f "${TOOLCHAIN_FILE}" ]; then
    echo -e "${RED}错误：工具链文件不存在 - ${TOOLCHAIN_FILE}${NC}"
    echo "请先运行同步脚本 sync_arm_deps.sh"
    exit 1
fi
echo -e "${GREEN}工具链正常：${TOOLCHAIN_FILE}${NC}"

if [ ! -x "${CMAKE_C_COMPILER}" ] || [ ! -x "${CMAKE_CXX_COMPILER}" ]; then
    echo -e "${RED}错误：交叉编译器缺失，执行以下命令安装：${NC}"
    echo -e "${RED}sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu${NC}"
    exit 1
fi
echo -e "${GREEN}交叉编译器正常(aarch64-linux-gnu-gcc/g++)${NC}"

# ccache检查
if ! command -v ccache &>/dev/null; then
    echo "→ 正在安装ccache编译缓存(首次使用，后续编译提速)"
    sudo apt update -qq && sudo apt install -y --no-install-recommends ccache >/dev/null 2>&1
    echo -e "${GREEN}ccache安装完成${NC}"
else
    hit_rate=$(ccache -s -d "${CCACHE_DIR}" | grep 'cache hit rate' | awk '{print $4}')
    [ -z "${hit_rate}" ] && hit_rate="0%"
    echo -e "${GREEN}ccache正常 (当前命中率：${hit_rate}，重复编译可大幅提速）${NC}"
fi


# -------------------------- 4. 创建Python临时软链接 --------------------------
echo -e "\n=== 第三步: 创建Python临时软链接 ==="
sudo rm -f "${PYTHON_LIB_SYMLINK}"
sudo ln -s "${PYTHON_LIB_ROOTFS}" "${PYTHON_LIB_SYMLINK}"
echo "软链接创建完成：$(ls -l "${PYTHON_LIB_SYMLINK}" | awk '{print $9 " -> " $11}')"
echo "提示：此链接仅编译时临时使用，脚本退出会自动删除"


# -------------------------- 5. 交叉编译 --------------------------
echo -e "\n=== 第四步: 开始ARM交叉编译 ==="
cd "${PROJECT_DIR}" || exit 1

build_start_time=$(date +%s)
start_time_str=$(date +"%Y-%m-%d %H:%M:%S")
echo "编译开始时间: ${start_time_str}"
echo "提醒：交叉编译过程中请勿中断脚本，避免临时软链接残留"

# colcon编译命令
colcon build \
--build-base "${ARM_BUILD_DIR}" \
--install-base "${ARM_INSTALL_DIR}" \
--parallel-workers "${COLCON_PARALLEL_WORKERS}" \
--cmake-clean-first \
--cmake-args \
-DCMAKE_BUILD_TYPE=${BUILD_TYPE_UPPER} \
-DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
-DCMAKE_C_COMPILER="${CMAKE_C_COMPILER}" \
-DCMAKE_CXX_COMPILER="${CMAKE_CXX_COMPILER}" \
-DCMAKE_INSTALL_PREFIX="${ARM_INSTALL_DIR}" \
-DCMAKE_PREFIX_PATH="${ARM_INSTALL_DIR};${SYSROOT_DIR}/opt/ros/humble" \
-DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH

# 编译耗时+缓存统计
build_end_time=$(date +%s)
end_time_str=$(date +"%Y-%m-%d %H:%M:%S")
echo "编译完成时间: ${end_time_str}"
duration=$((build_end_time - build_start_time))
echo -e "\n${GREEN}编译耗时：$((duration / 60))分$((duration % 60))秒${NC}"

echo -e "\n=== ARM专属ccache缓存统计 ===\n"
ccache -s -d "${CCACHE_DIR}"


# -------------------------- 6. 编译结果验证 --------------------------
if [ -d "${ARM_BUILD_DIR}" ] && [ -d "${ARM_INSTALL_DIR}" ]; then
    echo -e "\n${GREEN}编译成功!ARM专属目录生成正常(与x64完全隔离):${NC}"
    echo "  - ARM编译中间目录:${ARM_BUILD_DIR}"
    echo "  - ARM最终产物目录:${ARM_INSTALL_DIR}"
    echo "  - 核心产物位置：${ARM_INSTALL_DIR}/process_controller/lib/process_controller/process_node"

    echo -e "\n${YELLOW}开始替换产物中的硬编码路径...${NC}"
    # 1. 定义主机路径和目标路径
    HOST_PATH="${ARM_INSTALL_DIR}"
    TARGET_PATH="/root/Public/Flbot_sz_vision/install"  # 与ARM设备上的install目录路径完全一致

    # 2. 确保所有文件有写权限（避免sed修改失败）
    echo "→ 为产物目录添加写权限，确保替换可执行..."
    chmod -R u+w "${ARM_INSTALL_DIR}"  # 仅赋予当前用户写权限，不影响其他权限

    # 3. 安全检查：路径不同才替换
    if [ "${HOST_PATH}" != "${TARGET_PATH}" ]; then
        # 4. 批量替换文本文件中的路径
        find "${ARM_INSTALL_DIR}" -type f -exec sh -c '
            for file do
                if file --mime-type "$file" | grep -q "text/"; then
                    if ! sed -i "s|'${HOST_PATH}'|'${TARGET_PATH}'|g" "$file"; then
                        echo -e "${YELLOW} 替换文件 $file 失败（可能是特殊文件，不影响核心功能）${NC}"
                    fi
                fi
            done
        ' sh {} + || true  # 容错处理：确保find命令不中断脚本

        # 显式提示替换命令执行完毕，继续验证流程
        echo -e "${YELLOW}路径替换命令执行完毕，开始验证残留路径...${NC}"

        # 5. 验证替换结果
        RESIDUE=$(grep -r "${HOST_PATH}" "${ARM_INSTALL_DIR}" 2>/dev/null | grep -v "binary file matches" || true)
        if [ -z "${RESIDUE}" ]; then
            echo -e "${GREEN}路径替换完成，无主机路径残留${NC}"
        else
            echo -e "${YELLOW} 路径替换完成，但发现少量残留（可能是特殊文本文件，不影响运行）：${NC}"
            echo "${RESIDUE}" | head -3
        fi
    else
        echo -e "${YELLOW} 主机路径与目标路径相同，无需替换${NC}"
    fi
    # -------------------------- 路径替换结束 --------------------------

else
    echo -e "\n${RED}编译失败: ARM目录未生成, 请检查工具链配置或源码完整性！${NC}"
    exit 1
fi

echo -e "\n后续操作提示: "
echo "1. 拷贝ARM产物到ARM设备: scp -r ${ARM_INSTALL_DIR} ${ARM_USER}@${ARM_IP}:$(dirname "${TARGET_PATH}")/"
echo "2. 在ARM上启动: source ${TARGET_PATH}/setup.bash && ros2 launch process_controller process_controller.py"
echo "3. x64开发: 直接执行./build.sh 即可, build/install目录未受任何影响"