#!/bin/bash
# 在主机上运行
echo "🚀 同步 install 目录到 X5..."
rsync -avz --progress /home/wx/servicerobot/x5/tros_ws/install root@10.0.40.78:/root/tros_ws
echo "✅ 同步完成!"