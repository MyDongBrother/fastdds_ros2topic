#!/bin/bash
set -e

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <CPU_TYPE: x86|arm|ndk>"
    exit 1
fi

CPU_TYPE="$1"

### 交叉编译环境设置
BUILD_SUBDIR=""
case "$CPU_TYPE" in
    arm|ARM)
        export PATH=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin:$PATH
        export LD_LIBRARY_PATH=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/lib:$LD_LIBRARY_PATH
        export CC=aarch64-linux-gnu-gcc
        export CXX=aarch64-linux-gnu-g++
        BUILD_SUBDIR="build_arm"
        LIB_SUBDIR="lib/arm"
        ;;
    x86|X86)
        export CC=gcc
        export CXX=g++
        BUILD_SUBDIR="build_x86"
        LIB_SUBDIR="lib/x86"
        ;;
    ndk|NDK)
        export NDK_HOME=/opt/android-ndk-r26b
        export PATH=$NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin:$PATH
        export CC=aarch64-linux-android31-clang
        export CXX=aarch64-linux-android31-clang++
        BUILD_SUBDIR="build_ndk"
        LIB_SUBDIR="lib/ndk"
        ;;
    *)
        echo "Unknown CPU type: $CPU_TYPE"
        exit 1
        ;;
esac

### 目录准备
BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IDL_FILE="$BASE_DIR/resource/idl/PolygonFrame.idl"
TEMP_DIR="$BASE_DIR/temp"
OUTPUT_DIR="$BASE_DIR/output"

mkdir -p "$TEMP_DIR"
mkdir -p "$OUTPUT_DIR/include"
mkdir -p "$OUTPUT_DIR/$LIB_SUBDIR"

### 使用 fastddsgen 生成
echo "生成 fastddsgen 文件..."
# fastddsgen -cs -example armLinux2.6gcc -replace -I "$BASE_DIR/resource/idl" -d "$TEMP_DIR" "$IDL_FILE"
fastddsgen -cs -example CMake -replace -I "$BASE_DIR/resource/idl" -d "$TEMP_DIR" "$IDL_FILE"

### 处理 CMakeLists.txt
cd $TEMP_DIR

CMAKE_FILE="CMakeLists.txt"
TMP_FILE="CMakeLists.tmp"

rm -f "$TMP_FILE"
while IFS= read -r line; do
    # 跳过 find_package 行
    if [[ $line =~ ^find_package\(fastcdr ]] || [[ $line =~ ^find_package\(fastrtps ]]; then
        continue
    fi

    # 写入新行
    echo "$line" >> "$TMP_FILE"
done < "$CMAKE_FILE"

# 在 TMP_FILE 找到 "# Find requirements" 行后插入新的 include/link 指定

case "$CPU_TYPE" in
    x86|X86)
sed -i '/# Find requirements/a \
# 指定 FastDDS 的 include 和 lib 目录\n\
set(FASTDDS_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../fastdds/x86/include")\n\
set(FASTDDS_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/../fastdds/x86/lib")\n\
\n\
include_directories(${FASTDDS_INCLUDE_DIR})\n\
link_directories(${FASTDDS_LIBRARY_DIR})' "$TMP_FILE"
        ;;
    ndk|NDK)
sed -i '/# Find requirements/a \
# 指定 FastDDS 的 include 和 lib 目录\n\
set(FASTDDS_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../fastdds/ndk/include")\n\
set(FASTDDS_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/../fastdds/ndk/lib")\n\
\n\
include_directories(${FASTDDS_INCLUDE_DIR})\n\
link_directories(${FASTDDS_LIBRARY_DIR})' "$TMP_FILE"
        ;;
    *)
        echo "Unknown CPU type: $CPU_TYPE"
        exit 1
        ;;
esac

# 替换原文件
mv "$TMP_FILE" "$CMAKE_FILE"

skip_target=""
skipping_exec=0
skipping_tll=0
lib_targets=()

while IFS= read -r line; do
    if [[ $line =~ ^add_executable\(([^[:space:]]+) ]]; then
        skip_target="${BASH_REMATCH[1]}"
        skipping_exec=1
        continue
    elif [[ $skipping_exec -eq 1 ]]; then
        [[ $line =~ \) ]] && skipping_exec=0
        continue
    fi

    if [[ -n $skip_target ]]; then
        if [[ $line =~ ^target_link_libraries\(${skip_target}[[:space:]] ]]; then
            skipping_tll=1
            continue
        elif [[ $skipping_tll -eq 1 ]]; then
            [[ $line =~ \) ]] && { skipping_tll=0; skip_target=""; }
            continue
        fi
    fi

    if [[ $line =~ ^add_library\(([^[:space:]]+_lib)[[:space:]] ]]; then
        libname="${BASH_REMATCH[1]}"
        lib_targets+=("$libname")
        line=$(echo "$line" | sed -E "s/^add_library\([[:space:]]*[^[:space:]]+[[:space:]]+/add_library($libname SHARED /; s/\)$/ ${libname%_lib}PubSubTypes.cxx)/")
    fi

    echo "$line" >> "$TMP_FILE"
done < "$CMAKE_FILE"

mv "$TMP_FILE" "$CMAKE_FILE"

# 添加 install 规则
cat >> "$CMAKE_FILE" <<EOF

set(CMAKE_INSTALL_PREFIX "\${CMAKE_SOURCE_DIR}/../output" CACHE PATH "Install path" FORCE)

install(TARGETS ${lib_targets[*]}
    LIBRARY DESTINATION $LIB_SUBDIR
    ARCHIVE DESTINATION $LIB_SUBDIR
    RUNTIME DESTINATION bin
)

EOF

for lib in "${lib_targets[@]}"; do
    base_name="${lib%_lib}"
    cat >> "$CMAKE_FILE" <<EOF
install(FILES \${CMAKE_CURRENT_SOURCE_DIR}/${base_name}.h
              \${CMAKE_CURRENT_SOURCE_DIR}/${base_name}PubSubTypes.h
        DESTINATION include)
EOF
done

### 编译安装
mkdir -p "$TEMP_DIR/$BUILD_SUBDIR"
cd "$TEMP_DIR/$BUILD_SUBDIR"
cmake ..
make -j$(nproc)
make install

### 清理
# rm -rf "$TEMP_DIR"

echo "✅ 编译并安装完成：$OUTPUT_DIR/$LIB_SUBDIR"

