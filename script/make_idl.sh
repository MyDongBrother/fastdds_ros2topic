#!/bin/bash
set -e

### 目录准备
BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IDL_FILE="$BASE_DIR/resource/idl/PolygonFrame.idl"
TEMP_DIR="$BASE_DIR/temp"
OUTPUT_DIR="$BASE_DIR/output"

mkdir -p "$TEMP_DIR"
mkdir -p "$OUTPUT_DIR/include"
mkdir -p "$OUTPUT_DIR/lib/x86"

### 使用 fastddsgen 生成
echo "生成 fastddsgen 文件..."
fastddsgen -cs -example CMake -replace -I "$BASE_DIR/resource/idl" -d "$TEMP_DIR" "$IDL_FILE"

### 处理cmake文件,生成动态库
cd $TEMP_DIR

CMAKE_FILE="CMakeLists.txt"
TMP_FILE="CMakeLists.tmp"

rm -f "$TMP_FILE"

# 临时变量记录要跳过的 target
skip_target=""
skipping_exec=0
skipping_tll=0
lib_targets=()

while IFS= read -r line; do
    # 删除对应的 add_executable
    if [[ $line =~ ^add_executable\(([^[:space:]]+) ]]; then
        skip_target="${BASH_REMATCH[1]}"
        echo "删除 add_executable(${skip_target} ... )"
        skipping_exec=1
        continue
    elif [[ $skipping_exec -eq 1 ]]; then
        if [[ $line =~ \) ]]; then
            skipping_exec=0
        fi
        continue
    fi

    # 删除对应的 target_link_libraries
    if [[ -n $skip_target ]]; then
        if [[ $line =~ ^target_link_libraries\(${skip_target}[[:space:]] ]]; then
            echo "删除 target_link_libraries(${skip_target} ... )"
            skipping_tll=1
            continue
        elif [[ $skipping_tll -eq 1 ]]; then
            if [[ $line =~ \) ]]; then
                skipping_tll=0
                skip_target=""
            fi
            continue
        fi
    fi

    # 修改 add_library(xx_lib ...) → add_library(xx_lib SHARED ...) 并记录库名
    if [[ $line =~ ^add_library\(([^[:space:]]+_lib)[[:space:]] ]]; then
        libname="${BASH_REMATCH[1]}"
        lib_targets+=("$libname")
        echo "替换 $libname 为 SHARED 库"
        line=$(echo "$line" | sed -E "s/^add_library\(($libname)[[:space:]]/add_library(\1 SHARED /")
    fi

    echo "$line" >> "$TMP_FILE"
done < "$CMAKE_FILE"

mv "$TMP_FILE" "$CMAKE_FILE"

# 在 CMakeLists.txt 末尾追加 install 规则
cat >> "$CMAKE_FILE" <<EOF

# 安装路径设置到 output 目录
set(CMAKE_INSTALL_PREFIX "\${CMAKE_SOURCE_DIR}/../output" CACHE PATH "Install path" FORCE)

# 安装所有 *_lib 动态库到 lib/x86
install(TARGETS ${lib_targets[*]}
    LIBRARY DESTINATION lib/x86
    ARCHIVE DESTINATION lib/x86
    RUNTIME DESTINATION bin
)

# 安装对应的头文件
EOF

for lib in "${lib_targets[@]}"; do
    # 提取去掉 _lib 的前缀
    base_name="${lib%_lib}"
    # 安装头文件
    cat >> "$CMAKE_FILE" <<EOF
install(FILES \${CMAKE_CURRENT_SOURCE_DIR}/${base_name}.h
              \${CMAKE_CURRENT_SOURCE_DIR}/${base_name}PubSubTypes.h
        DESTINATION include)
EOF
done

echo "✅ 更新完成：CMakeLists.txt (含 install 规则)"

### 编译安装
echo "开始编译..."
cd "$TEMP_DIR"
mkdir -p build && cd build
rm -rf *
cmake ..
make -j$(nproc)
make install
echo "✅ 编译并安装完成：$OUTPUT_DIR/lib/x86"

### 删除temp文件夹
rm -r $TEMP_DIR
