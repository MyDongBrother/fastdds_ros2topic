#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/msg/hello_world.hpp" //添加msg所需的头文件

using namespace std::chrono_literals;
using base_interfaces::msg::HelloWorld; // 引入命名空间 base_interfaces::msg 中的 Student 类型。

// 修改节点名TopicSub
class TopicSub : public rclcpp::Node
{
public:
    // 构造函数
    // 1.创建节点时初始化节点名为TopicSub_node
    TopicSub() : Node("TopicSub_node")
    {
        subscription_ = this->create_subscription<HelloWorld>(
            "hello_world",
            10,
            std::bind(&TopicSub::topic_callback, this, std::placeholders::_1));
    }

private:
    // 回调函数
    void topic_callback(const HelloWorld::SharedPtr msg)
    {
        // 在终端中打印出订阅到的信息
        RCLCPP_INFO(this->get_logger(), "订阅的学生消息:name=%s", msg->data.c_str());
    }
    // 数据成员
    // 创建智能指针rclcpp::Subscription<Student> 类型的订阅者对象，
    rclcpp::Subscription<HelloWorld>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}