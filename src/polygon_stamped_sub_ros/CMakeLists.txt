cmake_minimum_required(VERSION 3.5)
project(polygon_stamped_sub_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Fast DDS 安装路径
set(FASTDDS_INCLUDE_DIR "/opt/fastdds/include")
set(FASTDDS_LIB_DIR     "/opt/fastdds/lib")

# 源文件 + Fast DDS 生成文件
file(GLOB_RECURSE SRC_FILES
    src/*.cpp
    src/*.cxx
    generated/*.cxx
)

add_executable(polygon_stamped_sub ${SRC_FILES})

# ROS 2 依赖
ament_target_dependencies(polygon_stamped_sub
    rclcpp
    geometry_msgs
)

# Fast DDS include + generated 头文件
target_include_directories(polygon_stamped_sub PUBLIC
    ${FASTDDS_INCLUDE_DIR}
    /home/bmi-jiandong/workspace/fastdds_ros2topic/generated
    /home/bmi-jiandong/workspace/fastdds_ros2topic/generated/geometry_msgs/msg
)

# Fast DDS 库
target_link_directories(polygon_stamped_sub PUBLIC
    ${FASTDDS_LIB_DIR}
)

target_link_libraries(polygon_stamped_sub
    fastrtps
    fastcdr
)

install(TARGETS
  polygon_stamped_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
