#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"

class EmcJoyNode : public rclcpp::Node
{
public:
    EmcJoyNode() : Node("emc_joy"), emc_button_pressed_(false)  // メンバー変数の初期化　初期状態はfalse
    {
        // サービスクライアントを作成
        emergency_client_ = this->create_client<std_srvs::srv::Trigger>("emergency");

        // Joyトピックのサブスクライバを作成
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "emc/joy", 10,
            std::bind(&EmcJoyNode::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // buttons[0]の状態をチェック
        if (msg->buttons.size() > 0)
        {
            if (msg->buttons[0] == 0 && emc_button_pressed_)
            {
                // ボタンが押された瞬間にリクエストを送信
                if (!emergency_client_->wait_for_service(std::chrono::seconds(1)))
                {
                    RCLCPP_WARN(this->get_logger(), "Emergency service is not available");
                    return;
                }

                // サービスリクエストを非同期で送信
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                emergency_client_->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Emergency service is called");

                // ボタンが離された状態に変更
                emc_button_pressed_ = false;
            }
            else if (msg->buttons[0] == 1 && !emc_button_pressed_)
            {
                // ボタンが離されたら、次に押されたときにリクエストを送る準備をする
                emc_button_pressed_ = true;
                RCLCPP_INFO(this->get_logger(), "If Emergency Button was released, Please push Robot's power button.");
            }
        }
    }

    // サービスクライアント
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_client_;

    // Joyメッセージのサブスクライバ
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    // 緊急停止ボタンの状態を記憶するフラグ
    bool emc_button_pressed_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmcJoyNode>());
    rclcpp::shutdown();
    return 0;
}
