#!/bin/bash

# === IHB-X5 底层库编译脚本 (线程安全优化版本) ===
echo "=== IHB-X5 底层库编译脚本 ==="
echo "线程安全优化版本"
echo "=============================="

# 设置编译目录
BUILD_DIR="build-angle"

# 清理之前的编译结果
echo "🧹 清理之前的编译结果..."
rm -rf $BUILD_DIR

# 创建build目录并进入
mkdir -p $BUILD_DIR

# 运行cmake配置
echo "⚙️  配置CMake..."
cmake ./demo -B $BUILD_DIR -DCMAKE_BUILD_TYPE=Release

# 检查cmake是否成功
if [ $? -ne 0 ]; then
    echo "❌ CMake配置失败！"
    exit 1
fi

# 编译项目
echo "🔨 开始编译 (使用 $(nproc) 个并行任务)..."
cmake --build $BUILD_DIR -j$(nproc)

# 检查编译是否成功
if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
    echo "📁 可执行文件位于: $BUILD_DIR/"

    # 列出生成的可执行文件
    echo "📋 生成的测试程序:"
    ls -la $BUILD_DIR/*.out 2>/dev/null || echo "   没有找到 .out 文件"
    ls -la $BUILD_DIR/hallTest $BUILD_DIR/step_motor $BUILD_DIR/serAngle $BUILD_DIR/canAngleTestThread 2>/dev/null

    echo ""
    echo "🚀 运行测试程序:"
    echo "   cd $BUILD_DIR && ./hallTest          - 霍尔传感器测试 (线程安全优化)"
    echo "   cd $BUILD_DIR && ./step_motor        - 步进电机测试 (线程安全优化)"
    echo "   cd $BUILD_DIR && ./serAngle          - 伺服电机+霍尔传感器综合测试 (性能优化)"
    echo "   cd $BUILD_DIR && ./canAngleTestThread - CAN通信多线程测试 (稳定性优化)"
    echo ""
    echo "💡 所有测试程序都已适配新的线程安全接口"
    echo "💡 按 Ctrl+C 可随时安全退出任何测试程序"
else
    echo "❌ 编译失败！"
    echo "请检查编译错误信息并修复后重试"
    exit 1
fi