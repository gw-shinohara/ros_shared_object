#include <cstdlib>
#include <cstring>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class Talker
{
public:
    Talker()
    {
        rclcpp::init(0, nullptr);

        node_ = std::make_shared<rclcpp::Node>("ros_shared_object_talker_node");
        rclcpp::QoS qos(rclcpp::KeepLast(7));

        pub_ = node_->create_publisher<sensor_msgs::msg::Image>("image_topic", qos);
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

    void talk(const cv::Mat &frame)
    {
        if (frame.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Received empty frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(*msg);
    
        RCLCPP_INFO(node_->get_logger(),
            "Published message address: %p", &frame
        );
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
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

    void talk(intptr_t ptr, const cv::Mat &frame)
    {
        reinterpret_cast<Talker*>(ptr)->talk(frame);
    }

    void destroy(intptr_t ptr)
    {
        delete reinterpret_cast<Talker*>(ptr);
    }
}
