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

        m_node = std::make_shared<rclcpp::Node>("ros_shared_object_talker_node");

        rclcpp::QoS qos(rclcpp::KeepLast(7));
        m_pub = m_node->create_publisher<std_msgs::msg::String>("topic", qos);
    }

    ~Talker()
    {
        rclcpp::shutdown();
    }

    void spin_some()
    {
        rclcpp::spin_some(m_node);
    }

    // publish message
    void talk(const int &count)
    {
        m_msg = std::make_unique<std_msgs::msg::String>();
        m_msg->data = "Hello from shared object: " + std::to_string(count);
        RCLCPP_INFO(m_node->get_logger(), "Publishing: '%s'", m_msg->data.c_str());

        m_pub->publish(std::move(m_msg));
    }

protected:
    static void _bind_methods();

    // replace rclcpp::Node with your custom node
    std::shared_ptr<rclcpp::Node> m_node;

    // publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;

    // message to publish
    std::unique_ptr<std_msgs::msg::String> m_msg;
};

namespace
{
    const char *AllocString(const char *str)
    {
        char *buf = static_cast<char *>(malloc(strlen(str) + 1));
        strcpy(buf, str);
        return buf;
    }
}

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
