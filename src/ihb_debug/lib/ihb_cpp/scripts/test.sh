#!/bin/bash

# 创建build目录并进入
mkdir -p build-test

# 运行cmake和make
cmake ./test -B build-test
cmake --build build-test

# 运行程序
# ./build-single/single