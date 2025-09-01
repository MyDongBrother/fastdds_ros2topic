#include <cstdio>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "PolygonFrameSubscriber.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"

class ROS2PolygonPublisher : public rclcpp::Node
{
public:
    ROS2PolygonPublisher() : Node("ros2_polygon_publisher")
    {
        curpos_pub_ =
            this->create_publisher<geometry_msgs::msg::PolygonStamped>(
                "/CurPos", 10);
        slot_pub_ =
            this->create_publisher<geometry_msgs::msg::PolygonStamped>(
                "/SlotInfo", 10);
    }

    void PublishFootprints(geometry_msgs::msg::PolygonStamped &poly_msg)
    {
        if (poly_msg.header.frame_id == "CurPos")
        {
            poly_msg.header.frame_id = "map";
            curpos_pub_->publish(poly_msg);
        }
        else if (poly_msg.header.frame_id == "SlotInfo")
        {
            poly_msg.header.frame_id = "map";
            slot_pub_->publish(poly_msg);
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr curpos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr slot_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROS2PolygonPublisher>();

    PolygonFrameSubscriber mysub;
    if (!mysub.init())
    {
        std::cout << "FastDDS init fail." << std::endl;
        return 1;
    }

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        geometry_msgs::msg::PolygonStamped ros_msg;
        mysub.run(ros_msg); // 填充 frame_id ("CurPos" 或 "SlotInfo")
        node->PublishFootprints(ros_msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
