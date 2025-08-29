#!/bin/bash

IDL_ROOT="./idl"
INCLUDE_ROOT="./include"

# 遍历所有idl文件
find "$IDL_ROOT" -name "*.idl" | while read -r idl_file; do
    # 提取包名和消息类型路径
    rel_path="${idl_file#$IDL_ROOT/}"      # geometry_msgs/msg/PolygonStamped.idl
    dir_path=$(dirname "$rel_path")       # geometry_msgs/msg
    file_name=$(basename "$idl_file")     # PolygonStamped.idl
    base_name="${file_name%.idl}"         # PolygonStamped

    # 创建对应的include文件夹
    out_dir="$INCLUDE_ROOT/$dir_path"
    mkdir -p "$out_dir"

    # 执行fastddsgen
    echo "Generating $idl_file -> $out_dir"
    fastddsgen -cs -typeros2 -I "$IDL_ROOT" -d "$out_dir" "$idl_file"
done

# 清理include中多余文件
echo "Cleaning up include directory..."
find "$INCLUDE_ROOT" -type f | while read -r inc_file; do
    # 生成对应的idl路径
    rel_path="${inc_file#$INCLUDE_ROOT/}"
    base_name=$(basename "$rel_path" | sed 's/\(PubSubTypes\)\?\.\(h\|cxx\)//g')
    idl_file_candidate="$IDL_ROOT/$(dirname "$rel_path")/$base_name.idl"

    if [ ! -f "$idl_file_candidate" ]; then
        echo "Removing extra file: $inc_file"
        rm -f "$inc_file"
    fi
done
