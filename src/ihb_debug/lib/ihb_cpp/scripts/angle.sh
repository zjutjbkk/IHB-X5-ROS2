#!/bin/bash

# === IHB-X5 底层库编译脚本 (线程安全优化版本) ===
echo "=== IHB-X5 底层库编译脚本 ==="
echo "线程安全优化版本"
echo "=============================="

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."     # 因为 scripts 是 ihb_cpp 的子目录
DEMO_DIR="$PROJECT_ROOT/demo"     # demo 在 ihb_cpp/demo

echo "📁 脚本目录: $SCRIPT_DIR"
echo "📁 项目根目录: $PROJECT_ROOT"
echo "📁 Demo 源码目录: $DEMO_DIR"

# 检查 demo 目录是否存在
if [ ! -d "$DEMO_DIR" ]; then
    echo "❌ 错误: demo 目录不存在！请检查路径: $DEMO_DIR"
    exit 1
fi

# 设置编译目录（可放在脚本同级或项目根目录）
BUILD_DIR="$PROJECT_ROOT/build-angle"

# 清理之前的编译结果
echo "🧹 清理之前的编译结果..."
rm -rf "$BUILD_DIR"

# 创建 build 目录
mkdir -p "$BUILD_DIR"

# 运行 cmake 配置（使用绝对路径）
echo "⚙️  配置 CMake..."
echo "📁 使用的 CMakeLists.txt 路径: $DEMO_DIR/CMakeLists.txt"
cmake "$DEMO_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release

# ========================
# ✅ 第二步：运行 cmake --build 进行实际编译（这才是生成可执行文件的关键！）
# ========================
echo "🔨 开始编译 (使用 $(nproc) 个并行任务)..."
cmake --build "$BUILD_DIR" -j$(nproc)


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
#    echo "   cd $BUILD_DIR && ./hallTest          - 霍尔传感器测试 (线程安全优化)"
    echo "   cd $BUILD_DIR && ./step_motor        - 步进电机测试 (线程安全优化)"
#    echo "   cd $BUILD_DIR && ./serAngle          - 伺服电机+霍尔传感器综合测试 (性能优化)"
#    echo "   cd $BUILD_DIR && ./canAngleTestThread - CAN通信多线程测试 (稳定性优化)"
    echo ""
    echo "💡 所有测试程序都已适配新的线程安全接口"
    echo "💡 按 Ctrl+C 可随时安全退出任何测试程序"
else
    echo "❌ 编译失败！"
    echo "请检查编译错误信息并修复后重试"
    exit 1
fi