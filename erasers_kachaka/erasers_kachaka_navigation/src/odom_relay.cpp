#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomQosRelay : public rclcpp::Node
{
public:
  OdomQosRelay()
  : Node("odom_qos_relay")
  {
    // パラメータの宣言（デフォルト値設定）
    this->declare_parameter("input_topic", "odometry/odometry");
    this->declare_parameter("output_topic", "odometry/odometry_reliable");

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    rclcpp::QoS pub_qos(1);
    pub_qos.reliable();
    pub_qos.durability_volatile();

    rclcpp::QoS sub_qos = rclcpp::SensorDataQoS();
    sub_qos.keep_last(10);

    // Publisher 作成
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, pub_qos);

    // Subscriber 作成
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      sub_qos,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) {
        this->publisher_->publish(std::move(msg));
      });

    RCLCPP_INFO(this->get_logger(), "Relaying %s -> %s with RELIABLE QoS", input_topic.c_str(), output_topic.c_str());
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomQosRelay>());
  rclcpp::shutdown();
  return 0;
}
