cmake_minimum_required(VERSION 3.5)
project(polygon_stamped_sub_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# 头文件路径
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../output/include
)
# 搜索所有动态库
file(GLOB THIRD_PARTY_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/../../output/lib/x86/*.so")
# 找 Fast DDS
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

add_executable(polygon_stamped_sub_ros
    src/PolygonFrameSubscriber.cxx
    src/PolygonFramePubSubMain.cxx
)

# 依赖 ROS2
ament_target_dependencies(polygon_stamped_sub_ros
    rclcpp
    geometry_msgs
)

# 链接 Fast DDS 库
target_link_libraries(polygon_stamped_sub_ros
    fastrtps
    fastcdr
    ${THIRD_PARTY_LIBS}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
