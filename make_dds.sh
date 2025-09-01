#!/bin/bash
set -e

# 如果 build_dds 已存在就直接用它
mkdir -p build_dds
cd build_dds

# 改成正确路径
cmake ../src/polygon_stamped_pub/
make

# 运行可执行文件
./polygon_stamped_pub
