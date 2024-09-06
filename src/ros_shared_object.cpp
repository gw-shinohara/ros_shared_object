#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RosSharedNode : public rclcpp::Node {
public:
    RosSharedNode() : Node("ros_shared_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        RCLCPP_INFO(this->get_logger(), "Node has been created");
    }

    // メッセージをパブリッシュする関数
    void publish_message(const std::string& msg_content) {
        auto message = std_msgs::msg::String();
        message.data = msg_content;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    }

    void run() {
        rclcpp::spin_some(shared_from_this());
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
