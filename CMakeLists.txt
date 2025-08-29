cmake_minimum_required(VERSION 3.8)
project(apa_fastdds)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 源文件：src + generated
file(GLOB_RECURSE SRC_SOURCES_FILES
     "src/*.cpp" "src/*.cxx" "src/*.c"
     "generated/*.cpp" "generated/*.cxx")

add_executable(apa_fastdds ${SRC_SOURCES_FILES})

target_link_libraries(apa_fastdds fastrtps fastcdr)

target_include_directories(apa_fastdds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated>
  $<INSTALL_INTERFACE:generated>)
