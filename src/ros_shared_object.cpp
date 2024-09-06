#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class RosSharedObjectLibrary : public rclcpp::Node {
public:
    RosSharedObjectLibrary() : Node("ros_shared_object_library_node") {
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

    // ノードを開始する関数
    void start() {
        rclcpp::spin_some(shared_from_this());
    }

    // ノードを停止する関数
    void stop() {
        rclcpp::shutdown();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

// ファクトリ関数の定義
extern "C" {
    RosSharedObjectLibrary* create_node() {
        return new RosSharedObjectLibrary();
    }

    void destroy_node(RosSharedObjectLibrary* node) {
        delete node;
    }
}
