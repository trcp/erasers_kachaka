#include "erasers_kachaka_cartographer/occupancy_grid_map_creator.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto processor = std::make_shared<erasers_kachaka_cartographer::OccupancyGridMapCreator>();
  rclcpp::spin(processor);
  rclcpp::shutdown();
  return 0;
}