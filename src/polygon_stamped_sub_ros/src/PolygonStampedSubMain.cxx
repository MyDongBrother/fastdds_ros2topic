#include <cstdio>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "PolygonStampedSubscriber.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"

class ROS2PolygonPublisher : public rclcpp::Node
{
public:
    ROS2PolygonPublisher() : Node("ros2_polygon_publisher")
    {
        goal_footprint_pub_ =
            this->create_publisher<geometry_msgs::msg::PolygonStamped>(
                "/global_costmap/start_footprint", 10);
    }

    void PublishFootprints(const geometry_msgs::msg::PolygonStamped &poly_msg)
    {
        goal_footprint_pub_->publish(poly_msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr goal_footprint_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROS2PolygonPublisher>();

    PolygonStampedSubscriber mysub;
    if (!mysub.init())
    {
        std::cout << "FastDDS init fail." << std::endl;
        return 1;
    }

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        geometry_msgs::msg::PolygonStamped ros_msg;
        mysub.run(ros_msg); // 返回自定义 PolygonStamped 数据结构

        node->PublishFootprints(ros_msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}