#!/bin/bash

# 创建build目录并进入
mkdir -p build-arm

# 运行cmake和make
cmake ./arm -B ./build-arm
cmake --build ./build-arm

# 运行程序
# ./build-single/single