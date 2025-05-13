#include <rclcpp/rclcpp.hpp>  // ROS2 の基本ノードクラス
#include <geometry_msgs/msg/twist.hpp>  // 速度コマンド用のメッセージ型
#include <termios.h>  // ターミナル設定を変更するためのライブラリ
#include <unistd.h>   // 標準入出力などUNIX系の関数
#include <iostream>
#include <algorithm>  // min/max関数のため

// ROS2ノードクラスの定義
class KeyboardTeleopNode : public rclcpp::Node
{
public:
  // コンストラクタ（ノード初期化）
  KeyboardTeleopNode()
  : Node("keyboard_teleop"),
    linear_speed_(0.25),  // 初期直進速度
    angular_speed_(0.5)   // 初期回転速度
  {
    // パブリッシャを作成（geometry_msgs::msg::Twist型のメッセージを送る）
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("manual_control/cmd_vel", 10);

    // 起動時の案内ログ
    RCLCPP_INFO(this->get_logger(), "Keyboard Teleop Node Started.");

    // ターミナル入力を非カノニカルモードに設定（キー入力の即時取得）
    configure_terminal();

    // 一定周期ごとにキーを読み取り、Twistメッセージを送るタイマー
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 100ミリ秒ごと
      std::bind(&KeyboardTeleopNode::publish_cmd, this)  // コールバック関数を登録
    );
  }

  // デストラクタ（終了時にターミナル設定を戻す）
  ~KeyboardTeleopNode()
  {
    restore_terminal();
  }

private:
  // ROS2パブリッシャ
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;
  // 送信するTwistメッセージ
  geometry_msgs::msg::Twist twist_;
  // 現在の移動速度
  double linear_speed_;
  double angular_speed_;

  // キー入力を読み取り、Twistメッセージを作成してパブリッシュ
  void publish_cmd()
  {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) < 0) {
      return;  // キー入力が無ければ何もしない
    }

    twist_ = geometry_msgs::msg::Twist();  // 速度指令の初期化

    switch (c) {
      case 'w':  // 前進
        twist_.linear.x = linear_speed_;
        break;
      case 's':  // 後退
        twist_.linear.x = -linear_speed_;
        break;
      case 'a':  // 左旋回
        twist_.angular.z = angular_speed_;
        break;
      case 'd':  // 右旋回
        twist_.angular.z = -angular_speed_;
        break;
      case 'q':  // 終了
        rclcpp::shutdown();
        return;
      default:
        return;  // 未対応キー
    }

    // Twist メッセージをパブリッシュ
    publisher_->publish(twist_);
  }

  struct termios oldt_;  // ターミナル設定の保存用

  // ターミナルを非カノニカルモードに設定（キー入力を即時取得、エコーOFF）
  void configure_terminal()
  {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt_);  // 現在の設定を取得
    newt = oldt_;
    newt.c_lflag &= ~(ICANON | ECHO);  // 行バッファ無効化、入力を表示しない
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 新しい設定を適用
  }

  // ターミナル設定を元に戻す
  void restore_terminal()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
  }
};

// エントリーポイント
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  // ROS2初期化
  rclcpp::spin(std::make_shared<KeyboardTeleopNode>());  // ノード実行（ループ）
  rclcpp::shutdown();  // シャットダウン処理
  return 0;
}