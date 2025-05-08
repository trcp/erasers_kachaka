#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <algorithm>

class PotFieldsNode : public rclcpp::Node {
public:
    PotFieldsNode() : Node("pot_fields_node") {
        // パラメータの宣言と取得
        this->declare_parameter("kr", 2.5);
        this->declare_parameter("d0", 0.5);
        this->declare_parameter("max_angular", 1.5);
        this->declare_parameter("valid_angle_min", -M_PI_2);  // 前方±90度のみ有効
        this->declare_parameter("valid_angle_max", M_PI_2);
        
        kr_ = this->get_parameter("kr").as_double();
        d0_ = this->get_parameter("d0").as_double();
        max_angular_ = this->get_parameter("max_angular").as_double();
        valid_angle_min_ = this->get_parameter("valid_angle_min").as_double();
        valid_angle_max_ = this->get_parameter("valid_angle_max").as_double();

        // サブスクライバーとパブリッシャーの設定
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "lidar/scan", 10,
            std::bind(&PotFieldsNode::laser_callback, this, std::placeholders::_1));
            
        force_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "navigation/pot_fields/rejective_force", 10);
            
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "navigation/pot_fields/visualization_marker", 10);

        init_marker();
    }

private:
    void init_marker() {
        marker_.header.frame_id = "base_link";
        marker_.ns = "pot_fields";
        marker_.id = 0;
        marker_.type = visualization_msgs::msg::Marker::ARROW;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.scale.x = 0.1;
        marker_.scale.y = 0.2;
        marker_.scale.z = 0.1;
        marker_.color.a = 1.0;
        marker_.color.r = 1.0;
        marker_.color.g = 0.0;
        marker_.color.b = 0.0;
        marker_.pose.orientation.w = 1.0;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float total_torque = 0.0;
        valid_points_count_ = 0;

        // 有効角度範囲の動的計算
        const float angle_min = msg->angle_min;
        const float angle_max = msg->angle_max;
        const float angle_increment = msg->angle_increment;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const float r = msg->ranges[i];
            if (std::isinf(r) || std::isnan(r) || r > d0_) continue;

            // 動的角度計算
            const float current_angle = angle_min + angle_increment * i;
            
            // 有効角度範囲チェック
            if (!is_valid_angle(current_angle)) continue;

            // 後方データチェック (ロボット構造による遮蔽)
            if (is_occluded_area(current_angle)) continue;

            // 反発力計算
            const float force = kr_ * (1.0/r - 1.0/d0_);
            total_torque += force * (-current_angle); // 角度に比例したトルク
            valid_points_count_++;
        }

        // 正規化処理
        if (valid_points_count_ > 0) {
            total_torque /= valid_points_count_;
        }

        // 角速度制限
        total_torque = std::clamp(total_torque, -max_angular_, max_angular_);

        publish_output(total_torque);
    }

    bool is_valid_angle(float angle) {
        // 角度を[-π, π]に正規化
        angle = std::fmod(angle + M_PI, 2*M_PI) - M_PI;
        return (angle >= valid_angle_min_ && angle <= valid_angle_max_);
    }

    bool is_occluded_area(float angle) {
        // 後方±30度を遮蔽領域と仮定
        const float occlusion_start = M_PI - M_PI/6;
        const float occlusion_end = -M_PI + M_PI/6;
        return (angle > occlusion_start || angle < occlusion_end);
    }

    void publish_output(float torque) {
        // トルクメッセージのパブリッシュ
        std_msgs::msg::Float32 force_msg;
        force_msg.data = torque;
        force_pub_->publish(force_msg);

        // マーカーの更新
        marker_.header.stamp = this->now();
        marker_.pose.position.x = 0.5 * torque;
        marker_pub_->publish(marker_);
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker marker_;
    
    float kr_;
    float d0_;
    float max_angular_;
    float valid_angle_min_;
    float valid_angle_max_;
    size_t valid_points_count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PotFieldsNode>());
    rclcpp::shutdown();
    return 0;
}