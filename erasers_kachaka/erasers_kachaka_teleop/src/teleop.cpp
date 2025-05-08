#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

bool emergency_flag = false;

class JoyTranslate : public rclcpp::Node
{
public:
    JoyTranslate() : Node("joy_translate_node")
    {
        // パブリッシャーとサブスクライバーの作成
        pub = this->create_publisher<geometry_msgs::msg::Twist>("manual_control/cmd_vel", 10);
        sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoyTranslate::callback, this, std::placeholders::_1));
    }

    void callback(const sensor_msgs::msg::Joy::SharedPtr sub_joy)
    {
        auto cmd_vel = geometry_msgs::msg::Twist();
        if (sub_joy->buttons[4])
        {
            cmd_vel.linear.x = sub_joy->axes[1] / 8.0;
            cmd_vel.angular.z = sub_joy->axes[0];
            if (sub_joy->buttons[6])
            {
                cmd_vel.linear.x = sub_joy->axes[1];
                cmd_vel.angular.z = sub_joy->axes[0] * 2;
            }

            pub->publish(cmd_vel);
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyTranslate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
