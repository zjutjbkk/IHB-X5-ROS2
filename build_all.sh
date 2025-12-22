#!/bin/bash

# ==============================================================================
#           【IHB】Docker交叉编译-完整构建脚本 (All-in-One)
# ==============================================================================
#
# 功能: 在主机上运行，命令指定的Docker容器，对整个ROS2工作区进行完整的交叉编译。
#       该脚本被设计为可移植的，会自动推断其所在的路径。
#
# 作者: wx & AI Assistant
# 版本: v2.0
#
# 用法:
#   在您的仓库目录下直接运行: ./build_all.sh
#
# ==============================================================================

# --- 配置区 (通常无需修改) ---
CONTAINER_NAME="x5-dev-env"                  # 您用 --name 创建的容器名
WORKSPACE_DIR_IN_CONTAINER="/mnt/test/tros_ws" # 容器内的工作区路径

# --- 智能路径推断 (核心) ---
# SCRIPT_DIR: 获取脚本文件本身所在的绝对路径
#             (例如: /home/wx/servicerobot/x5/tros_ws/src/ihb/IHB-X5-ROS2)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

# HOST_WORKSPACE_ROOT: 从脚本路径向上推断出tros_ws的绝对路径
#                      (例如: /home/wx/servicerobot/x5/tros_ws)
HOST_WORKSPACE_ROOT=$(cd "${SCRIPT_DIR}/../../../" &>/dev/null && pwd)

# --- 脚本主体 ---

# 1. 在主机上打印指挥信息
echo "=========================================================="
echo "  🚀  准备在主机上指挥容器进行【完整】编译..."
echo "=========================================================="
echo "    > 目标容器:             ${CONTAINER_NAME}"
echo "    > 主机工作区根目录:     ${HOST_WORKSPACE_ROOT}"
echo "    > 容器内工作区路径:     ${WORKSPACE_DIR_IN_CONTAINER}"
echo "    > 目标平台:             X5"
echo "----------------------------------------------------------"
echo ""

# 2. 检查主机工作区路径是否存在，以防万一
if [ ! -d "${HOST_WORKSPACE_ROOT}" ]; then
    echo "❌ 错误：无法在路径 '${HOST_WORKSPACE_ROOT}' 找到工作区。"
    echo "   请检查脚本是否放置在正确的仓库子目录中。"
    exit 1
fi

# 3. 构建要在容器内执行的命令字符串
#    由于不再需要编译特定包，命令变得非常简单。
COMMAND_TO_EXEC="cd ${WORKSPACE_DIR_IN_CONTAINER} && bash ./robot_dev_config/build.sh -p X5"

# 4. 【核心】使用 'docker exec' 执行命令
#    -i 保证标准输入开启，-t 分配一个伪终端，让输出更美观、颜色更友好。
sudo docker exec -it "${CONTAINER_NAME}" bash -c "${COMMAND_TO_EXEC}"

# 5. 检查上一步的命令是否执行成功，并给出最终反馈
#    $? 会获取上一个命令的退出码，0代表成功，其他代表失败。
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================================="
    echo "  ✅  完整编译命令在容器内成功完成！"
    echo "=========================================================="
else
    echo ""
    echo "=========================================================="
    echo "  ❌  完整编译命令在容器内执行失败，请检查上面的日志输出。"
    echo "=========================================================="
fi