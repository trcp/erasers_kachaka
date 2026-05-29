#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

bool emergency_flag = false;

class JoyTranslate : public rclcpp::Node
{
public:
    JoyTranslate() : Node("joy_translate_node")
    {
        // パラメータの宣言とデフォルト値の設定
        this->declare_parameter<int>("drive_button", 4);
        this->declare_parameter<int>("acc_button", 5);

        // 起動時に一度だけパラメータの値をロガーで出力
        RCLCPP_INFO(this->get_logger(), "Node initialized with drive_button: %ld, acc_button: %ld",
                    this->get_parameter("drive_button").as_int(),
                    this->get_parameter("acc_button").as_int());

        // パブリッシャーとサブスクライバーの作成
        pub = this->create_publisher<geometry_msgs::msg::Twist>("manual_control/cmd_vel", 10);
        sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoyTranslate::callback, this, std::placeholders::_1));
    }

    void callback(const sensor_msgs::msg::Joy::SharedPtr sub_joy)
    {
        // 実行中にパラメータが変更されても反映されるよう、コールバック内で最新のパラメータを取得
        int drive_button;
        int acc_button;
        this->get_parameter("drive_button", drive_button);
        this->get_parameter("acc_button", acc_button);

        // 指定されたボタンインデックスがJoyメッセージの配列範囲外にならないようチェック
        if (drive_button >= static_cast<int>(sub_joy->buttons.size()) || 
            acc_button >= static_cast<int>(sub_joy->buttons.size())) 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Button index out of range!");
            return;
        }

        auto cmd_vel = geometry_msgs::msg::Twist();

        // 操作有効ボタンが押されているか判定
        if (sub_joy->buttons[drive_button])
        {
            // 加速ボタンが追加で押されているか判定
            if (sub_joy->buttons[acc_button])
            {
                // 加速時の速度
                cmd_vel.linear.x = sub_joy->axes[1];
                cmd_vel.angular.z = sub_joy->axes[0] * 2;
            }
            else
            {
                // 通常時の速度
                cmd_vel.linear.x = sub_joy->axes[1] / 8.0;
                cmd_vel.angular.z = sub_joy->axes[0];
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