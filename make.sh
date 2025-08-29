#!/bin/bash
set -e

BUILD_DIR="build"

# 创建构建目录
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# 生成 Makefile
cmake ..

# 编译
make -j$(nproc)

# 可执行文件名，根据 CMakeLists.txt 改
EXECUTABLE="apa_fastdds"

# 运行程序
./$EXECUTABLE
