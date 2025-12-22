#!/bin/bash

# 创建build目录并进入
mkdir -p build-single

# 运行cmake和make
cmake ./single -B ./build-single
cmake --build ./build-single 

# 运行程序
# ./build-single/single