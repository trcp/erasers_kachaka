#include "erasers_kachaka_cartographer/occupancy_grid_map_creator.hpp"

namespace erasers_kachaka_cartographer {

OccupancyGridMapCreator::OccupancyGridMapCreator() 
  : Node("occupancy_grid_map_creator") 
{
  // パラメータ宣言
  declare_parameter("obstacle_threshold", 50);
  declare_parameter("publish_original", false);
  declare_parameter("input_topic", "/cartographer/map");
  declare_parameter("output_topic", "/map");

  // パラメータ取得
  obstacle_threshold_ = get_parameter("obstacle_threshold").as_int();
  publish_original_ = get_parameter("publish_original").as_bool();
  const auto input_topic = get_parameter("input_topic").as_string();
  const auto output_topic = get_parameter("output_topic").as_string();

  // QoS設定
  rclcpp::QoS map_qos(rclcpp::KeepLast(10));
  map_qos.reliable();
  map_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  // サブスクライバーとパブリッシャーの初期化
  sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    input_topic, map_qos,
    std::bind(&OccupancyGridMapCreator::process_grid, this, std::placeholders::_1));

  pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, map_qos);

  RCLCPP_INFO(get_logger(), "Start occupancy grid processing:");
  RCLCPP_INFO(get_logger(), " - Obstacle threshold: %d", obstacle_threshold_);
  RCLCPP_INFO(get_logger(), " - Publish original: %s", 
              publish_original_ ? "true" : "false");
}

void OccupancyGridMapCreator::process_grid(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  auto processed = *msg;
  
  if(!publish_original_) {
    std::transform(
      msg->data.begin(), msg->data.end(), processed.data.begin(),
      [this](int8_t val) -> int8_t {
        if(val == -1) return -1;  // 未知領域は維持
        return (val >= obstacle_threshold_) ? 100 : 0;
      }
    );
  }

  processed.header.stamp = now();
  processed.header.frame_id = "map";
  pub_->publish(processed);
}

}  // namespace erasers_kachaka_cartographer