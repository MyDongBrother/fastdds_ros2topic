cmake_minimum_required(VERSION 3.5)
project(polygon_stamped_pub)

# 头文件路径
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../output/include
)

# 搜索所有动态库
file(GLOB THIRD_PARTY_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/../../output/lib/x86/*.so")

# Fast DDS
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 源文件
file(GLOB_RECURSE SRC_SOURCES_FILES
     "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
     "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

# 可执行文件
add_executable(polygon_stamped_pub ${SRC_SOURCES_FILES})

# 链接库
target_link_libraries(polygon_stamped_pub
    fastrtps
    fastcdr
    ${THIRD_PARTY_LIBS}
)

target_include_directories(polygon_stamped_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated>
  $<INSTALL_INTERFACE:generated>)

# 安装
install(TARGETS polygon_stamped_pub DESTINATION bin)
