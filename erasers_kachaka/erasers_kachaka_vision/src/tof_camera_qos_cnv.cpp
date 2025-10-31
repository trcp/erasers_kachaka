#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rclcpp/qos.hpp"

using std::placeholders::_1;

class QosConverterNode : public rclcpp::Node
{
public:
  QosConverterNode() : Node("tof_camera_qos_conv_node")
  {
    RCLCPP_INFO(this->get_logger(), "QoS Converter Node starting...");

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data);
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(5));

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "tof_camera/image_raw", sub_qos,
      std::bind(&QosConverterNode::image_callback, this, _1));

    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "tof_camera/camera_info", sub_qos,
      std::bind(&QosConverterNode::info_callback, this, _1));

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(
      "tof_camera/reliable/image_raw", pub_qos);

    pub_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "tof_camera/reliable/camera_info", pub_qos);

    RCLCPP_INFO(this->get_logger(), "Subscribing to BEST_EFFORT: %s", sub_image_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Subscribing to BEST_EFFORT: %s", sub_info_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing to RELIABLE: %s", pub_image_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing to RELIABLE: %s", pub_info_->get_topic_name());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    pub_image_->publish(*msg);
  }

  void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    pub_info_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosConverterNode>());
  rclcpp::shutdown();
  return 0;
}
