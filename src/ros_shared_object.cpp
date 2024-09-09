#include <cstdlib>
#include <cstring>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Talkerクラスの定義と実装
class Talker
{
public:
    Talker()
    {
        // ノードとして機能するようにここでrclcppを初期化する。引数はダミー
        rclcpp::init(0, nullptr);

        // ノードを生成する
        node_ = std::make_shared<rclcpp::Node>("ros_shared_object_talker_node");
        rclcpp::QoS qos(rclcpp::KeepLast(7));

        // トピック名を"chatter"にすることで、ROS2のdemo_nodesのListenerで購読できるようになる
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
        // アプリケーション側から都度呼び出されることを想定
        rclcpp::spin_some(node_);
    }

    void talk(const int &count)
    {
        // 配信する
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
// アプリケーションからこの共有オブジェクトの機能にアクセスするためのエントリポイントの定義と実装
// manglingを避けるためにCリンケージにする

// Talkerのインスタンスを生成する
intptr_t create()
{
    return reinterpret_cast<intptr_t>(new Talker);
}

// Talkerのspin_someを呼び出す
void spin_some(intptr_t ptr)
{
    reinterpret_cast<Talker*>(ptr)->spin_some();
}

// Talkerのtalkを呼び出す
void talk(intptr_t ptr, int count)
{
    reinterpret_cast<Talker*>(ptr)->talk(count);
}

// Talkerのインスタンスを破棄する
void destroy(intptr_t ptr)
{
    delete reinterpret_cast<Talker*>(ptr);
}

}

