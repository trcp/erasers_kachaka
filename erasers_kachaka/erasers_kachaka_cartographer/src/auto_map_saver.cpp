#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include <chrono>
#include <thread>
#include <string>

class AutoMapSaver : public rclcpp::Node {
public:
    AutoMapSaver() : Node("auto_map_saver") {
        this->declare_parameter<std::string>("map_name", "map");
        this->declare_parameter<std::string>("map_save_path", "/home/roboworks/education_ws/src/erasers_kachaka/erasers_kachaka/erasers_kachaka_cartographer/map");
        this->declare_parameter<int>("save_late", 5);

        this->get_parameter("map_save_path", map_save_path_);
        this->get_parameter("map_name", map_name_);
        this->get_parameter("save_late", save_late_);

        namespace_ = this->get_namespace();

        timer_ = this->create_wall_timer(
            std::chrono::seconds(save_late_),
            std::bind(&AutoMapSaver::saveMap, this)
        );
    }

private:
    void saveMap() {
        std::string cmd = "ros2 run nav2_map_server map_saver_cli -f \"" + map_save_path_ + "/" + map_name_ + "\" --ros-args -p map_subscribe_transient_local:=true -r __ns:=" + namespace_;
        
        int result = std::system(cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved %s", map_save_path_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Map does not exist!");
        }
    }

    std::string map_save_path_;
    std::string map_name_;
    int save_late_;
    std::string namespace_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoMapSaver>());
    rclcpp::shutdown();
    return 0;
}
