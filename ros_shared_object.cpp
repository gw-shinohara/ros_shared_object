#include <cstdlib>
#include <cstring>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Reference
// https://github.com/flynneva/godot_ros/blob/main/include/godot_ros/demos/talker.hpp
class Talker
{
public:
    Talker()
    {
        rclcpp::init(0, nullptr);

        node_ = std::make_shared<rclcpp::Node>("ros_shared_object_talker_node");
        rclcpp::QoS qos(rclcpp::KeepLast(7));
        pub_ = node_->create_publisher<std_msgs::msg::String>("chatter", qos);
    }

    Talker(const Talker&) = delete;
    Talker& operator = (const Talker&) = delete;

    ~Talker()
    {
        rclcpp::shutdown();
    }

    void spin_some()
    {
        rclcpp::spin_some(node_);
    }

    void talk(const int &count)
    {
        msg_ = std::make_unique<std_msgs::msg::String>();
        msg_->data = "Hello from shared object: " + std::to_string(count);
        RCLCPP_INFO(node_->get_logger(), "Publishing: '%s'", msg_->data.c_str());
        pub_->publish(std::move(msg_));
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    std::unique_ptr<std_msgs::msg::String> msg_;
};

extern "C"
{

intptr_t create()
{
    return reinterpret_cast<intptr_t>(new Talker);
}

void spin_some(intptr_t ptr)
{
    reinterpret_cast<Talker*>(ptr)->spin_some();
}

void talk(intptr_t ptr, int count)
{
    reinterpret_cast<Talker*>(ptr)->talk(count);
}

void destroy(intptr_t ptr)
{
    delete reinterpret_cast<Talker*>(ptr);
}

}
