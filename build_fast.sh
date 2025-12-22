#!/bin/bash

# ==============================================================================
#           【IHB】Docker交叉编译-极速增量构建脚本 (build_fast.sh)
# ==============================================================================
#
# 功能: 在主机上运行，直接命令Docker容器内的colcon，只编译一个或多个
#       指定的包，以达到分钟级甚至秒级的最快编译速度。
#
# 作者: wx & AI Assistant
# 版本: v2.0
#
# 用法:
#   ./build_fast.sh <package_1> [package_2] [package_3] ...
#
# ==============================================================================

# --- 配置区 (通常无需修改) ---
CONTAINER_NAME="x5-dev-env"
WORKSPACE_DIR_IN_CONTAINER="/mnt/test/tros_ws"

# --- 脚本主体 ---

# 1. 检查是否至少提供了一个包名
if [ $# -eq 0 ]; then
    echo "❌ 错误: 请至少提供一个需要快速编译的包名。"
    echo "   用法: $0 <package_1> [package_2] ..."
    exit 1
fi

# 2. 获取所有传入的包名
PACKAGES_TO_BUILD="$@"

# 3. 【核心】构建要在容器内执行的命令字符串
#    - 我们手动为colcon提供所有必要的交叉编译参数。
#    - 所有路径都是相对于容器内部的，而不是主机！
COMMAND_TO_EXEC="
    echo '--- Sourcing ROS2 environment ---' && \
    source /opt/ros/humble/setup.bash && \
    echo '--- Entering workspace: ${WORKSPACE_DIR_IN_CONTAINER} ---' && \
    cd ${WORKSPACE_DIR_IN_CONTAINER} && \
    echo '--- Starting colcon build ---' && \
    colcon build \
        --packages-select ${PACKAGES_TO_BUILD} \
        --merge-install \
        --cmake-args \
            --no-warn-unused-cli \
            -DCMAKE_TOOLCHAIN_FILE=robot_dev_config/aarch64_toolchainfile.cmake \
            -DPLATFORM_X5=ON \
            -DTHIRDPARTY=ON \
            -DBUILD_TESTING=OFF
"

# 4. 在主机上打印指挥信息
echo "=========================================================="
echo "  ⚡️  准备在主机上指挥容器进行【极速增量编译】..."
echo "=========================================================="
echo "    > 目标容器:     ${CONTAINER_NAME}"
echo "    > 目标平台:     X5"
echo "    > 目标编译包:   ${PACKAGES_TO_BUILD}"
echo "----------------------------------------------------------"
echo ""

# 5. 【核心】使用 'docker exec' 执行命令
sudo docker exec -it "${CONTAINER_NAME}" bash -c "${COMMAND_TO_EXEC}"

# 6. 检查上一步的命令是否执行成功，并给出最终反馈
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================================="
    echo "  ✅  极速增量编译成功！"
    echo "=========================================================="
else
    echo ""
    echo "=========================================================="
    echo "  ❌  极速增量编译失败，请检查上面的日志输出。"
    echo "=========================================================="
fi