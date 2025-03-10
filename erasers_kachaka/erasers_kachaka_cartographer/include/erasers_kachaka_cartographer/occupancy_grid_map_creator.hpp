#ifndef ERASERS_KACHAKA_CARTOGRAPHER__OCCUPANCY_GRID_MAP_CREATOR_HPP
#define ERASERS_KACHAKA_CARTOGRAPHER__OCCUPANCY_GRID_MAP_CREATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace erasers_kachaka_cartographer {

class OccupancyGridMapCreator : public rclcpp::Node {
public:
  OccupancyGridMapCreator();

private:
  void process_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  int obstacle_threshold_;
  bool publish_original_;
};

}  // namespace erasers_kachaka_cartographer

#endif