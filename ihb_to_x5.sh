#!/bin/bash
# 在主机上运行
echo "🚀 同步 ihb 目录到 X5..."
rsync -avz --progress /home/wx/servicerobot/x5/tros_ws/src/ihb/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp root@10.0.40.78:/root/arm_control
echo "✅ 同步完成!"