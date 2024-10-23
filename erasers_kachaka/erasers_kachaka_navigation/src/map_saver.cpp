#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <string>
#include <stdexcept>

class AutoMapSaver : public rclcpp::Node
{
public:
    AutoMapSaver() : Node("auto_map_saver")
    {
        this->declare_parameter<std::string>("map_name", "map");
        this->declare_parameter<std::string>("map_save_path", "/home/ngsr/ros_ws/src/ngsr/ngsr_navigation/ngsr_navigation/map");
        this->declare_parameter<int>("save_late", 5);

        map_name_ = this->get_parameter("map_name").as_string();
        map_save_path_ = this->get_parameter("map_save_path").as_string();
        save_late_ = this->get_parameter("save_late").as_int();

        cmd_ = "ros2 run nav2_map_server map_saver_cli -f " + map_save_path_ + "/" + map_name_;

        save_map();
    }

private:
    void save_map()
    {
        while (rclcpp::ok())
        {
            try
            {
                int ret = system(cmd_.c_str());
                if (ret == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "saved %s", map_save_path_.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "map does not exist!");
                }

                std::this_thread::sleep_for(std::chrono::seconds(save_late_));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            }
        }
    }

    std::string map_name_;
    std::string map_save_path_;
    int save_late_;
    std::string cmd_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoMapSaver>());
    rclcpp::shutdown();
    return 0;
}
