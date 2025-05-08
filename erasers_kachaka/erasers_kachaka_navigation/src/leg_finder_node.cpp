#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

class LegFinderNode : public rclcpp::Node {
public:
    LegFinderNode() : Node("leg_finder_node") {
        // パラメータ宣言
        declare_parameters();
        
        // 変数初期化
        initialize_variables();
        
        // サービス設定
        setup_services();
        
        // パブリッシャー設定
        setup_publishers();
        
        RCLCPP_INFO(this->get_logger(), "Leg Finder Node initialized");
    }

private:
    // 定数定義
    const float FILTER_THRESHOLD = 0.081f;  // ノイズフィルタの閾値[m]
    const float FLANK_THRESHOLD = 0.04f;    // フランク検出閾値[m]
    const float PIERNA_DELGADA = 0.006241f; // 細い足の閾値(7.9cm)
    const float PIERNA_GRUESA = 0.037f;     // 太い足の閾値(19.23cm)
    const float DOS_PIERNAS_CERCAS = 0.022201f; // 近接した両足(14.9cm)
    const float DOS_PIERNAS_LEJOS = 0.16f;     // 離れた両足(40cm)
    const float MIN_DETECTION_RANGE = 0.4f;     // 最小検出距離[m]
    const float MAX_DETECTION_RANGE = 10.0f;    // 最大検出距離[m]

    // ROS2インターフェース
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr legs_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr legs_found_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    // フィルタ状態変数
    std::vector<float> legs_x_filter_input_;
    std::vector<float> legs_x_filter_output_;
    std::vector<float> legs_y_filter_input_;
    std::vector<float> legs_y_filter_output_;

    // 動作状態
    bool is_active_ = false;
    bool legs_found_ = false;
    std::string frame_id_;
    float occlusion_start_, occlusion_end_;

    // パラメータ宣言関数
    void declare_parameters() {
        // フレームID設定 (デフォルト: base_range_sensor_link)
        declare_parameter("frame_id", "laser_frame");
        
        // 遮蔽領域角度 (後方±150度)
        declare_parameter("occlusion_angle", M_PI * 5.0 / 6.0);

        // leg
        declare_parameter("min_leg_width", 0.15);
        declare_parameter("max_leg_width", 0.30);
        declare_parameter("leg_aspect_ratio", 1.5);
        
        // バターワースフィルタ係数
        declare_parameter("bfa0x", 1.0);
        declare_parameter("bfa1x", -1.760041880343169);
        declare_parameter("bfa2x", 1.182893262037831);
        declare_parameter("bfa3x", -0.278059917634546);
        declare_parameter("bfb0x", 0.018098933007514);
        declare_parameter("bfb1x", 0.054296799022543);
        declare_parameter("bfb2x", 0.054296799022543);
        declare_parameter("bfb3x", 0.018098933007514);
    }

    // 変数初期化関数
    void initialize_variables() {
        // パラメータ取得
        frame_id_ = get_parameter("frame_id").as_string();
        const float occlusion_angle = get_parameter("occlusion_angle").as_double();
        
        // 遮蔽領域計算
        occlusion_start_ = M_PI - occlusion_angle;
        occlusion_end_ = -M_PI + occlusion_angle;
        
        // フィルタバッファ初期化
        legs_x_filter_input_.resize(4, 0.0f);
        legs_x_filter_output_.resize(4, 0.0f);
        legs_y_filter_input_.resize(4, 0.0f);
        legs_y_filter_output_.resize(4, 0.0f);
    }

    // サービス設定関数
    void setup_services() {
        service_ = create_service<std_srvs::srv::SetBool>(
            "navigation/leg_finder/execute",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                is_active_ = req->data;
                manage_subscriptions();
                res->success = true;
                res->message = is_active_ ? "LegFinder activated" : "LegFinder deactivated";
                RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
            });
    }

    // パブリッシャー設定関数
    void setup_publishers() {
        legs_pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "navigation/leg_finder/leg_poses", rclcpp::SensorDataQoS());
            
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "navigation/leg_finder/marker", 10);
            
        legs_found_pub_ = create_publisher<std_msgs::msg::Bool>(
            "navigation/leg_finder/legs_found", rclcpp::SystemDefaultsQoS());
    }

    // サブスクリプション管理関数
    void manage_subscriptions() {
        if (is_active_ && !laser_sub_) {
            auto lidar_qos = rclcpp::QoS(
                rclcpp::KeepLast(5)  // パブリッシャーと同じキューサイズ
            );
            lidar_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            lidar_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

            laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "lidar/scan", lidar_qos,
                std::bind(&LegFinderNode::laser_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "LiDAR subscription started");
        } else if (!is_active_ && laser_sub_) {
            laser_sub_.reset();
            RCLCPP_INFO(this->get_logger(), "LiDAR subscription stopped");
        }
    }

    // LiDARコールバック関数
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // データ有効性チェック
        if (msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty LiDAR data received");
            return;
        }

        // データ前処理
        auto filtered_ranges = filter_laser_ranges(*msg);
        
        // 足候補検出
        std::vector<float> legs_x, legs_y;
        find_leg_hypothesis(*msg, filtered_ranges, legs_x, legs_y);
        
        // 追跡処理
        process_legs(legs_x, legs_y, msg);
    }

    // LiDARデータフィルタリング関数
    std::vector<float> filter_laser_ranges(const sensor_msgs::msg::LaserScan& scan) {
        std::vector<float> filtered(scan.ranges.size(), 0.0f);
        const size_t max_idx = scan.ranges.size() - 1;

        for(size_t i = 1; i < max_idx; ++i) {
            // 遮蔽領域チェック
            const float angle = scan.angle_min + i * scan.angle_increment;
            if(is_occluded(angle)) {
                filtered[i] = 0.0f;
                continue;
            }

            // 距離範囲チェック
            const float range = scan.ranges[i];
            if(std::isnan(range) || range < MIN_DETECTION_RANGE || range > MAX_DETECTION_RANGE) {
                filtered[i] = 0.0f;
                continue;
            }

            // 移動平均フィルタ
            if(std::abs(scan.ranges[i-1] - range) < FILTER_THRESHOLD &&
               std::abs(range - scan.ranges[i+1]) < FILTER_THRESHOLD) {
                filtered[i] = (scan.ranges[i-1] + range + scan.ranges[i+1]) / 3.0f;
            } else if(std::abs(scan.ranges[i-1] - range) < FILTER_THRESHOLD) {
                filtered[i] = (scan.ranges[i-1] + range) / 2.0f;
            } else {
                filtered[i] = 0.0f;
            }
        }
        return filtered;
    }

    // 遮蔽領域判定関数
    bool is_occluded(float angle) {
        // 角度を[-π, π]に正規化
        angle = std::fmod(angle + M_PI, 2*M_PI) - M_PI;
        return (angle > occlusion_start_ || angle < occlusion_end_);
    }

    // 足候補検出関数
    void find_leg_hypothesis(const sensor_msgs::msg::LaserScan& scan,
                            const std::vector<float>& filtered_ranges,
                            std::vector<float>& legs_x,
                            std::vector<float>& legs_y) {
        // デカルト座標変換
        std::vector<float> laser_x, laser_y;
        laser_x.reserve(scan.ranges.size());
        laser_y.reserve(scan.ranges.size());

        for(size_t i = 0; i < scan.ranges.size(); ++i) {
            if(filtered_ranges[i] <= 0.0f) continue;

            const float angle = scan.angle_min + i * scan.angle_increment;
            laser_x.push_back(filtered_ranges[i] * std::cos(angle));
            laser_y.push_back(filtered_ranges[i] * std::sin(angle));
        }

        // フランク検出
        std::vector<size_t> flank_indices;
        for(size_t i = 1; i < laser_x.size(); ++i) {
            if(std::hypot(laser_x[i] - laser_x[i-1], laser_y[i] - laser_y[i-1]) > FLANK_THRESHOLD) {
                flank_indices.push_back(i);
            }
        }

        // 足候補クラスタリング
        process_flanks(flank_indices, laser_x, laser_y, legs_x, legs_y);
    }

    // フランク処理関数
    void process_flanks(const std::vector<size_t>& flank_indices,
                       const std::vector<float>& laser_x,
                       const std::vector<float>& laser_y,
                       std::vector<float>& legs_x,
                       std::vector<float>& legs_y) {
        for(size_t i = 1; i < flank_indices.size(); ++i) {
            const size_t start = flank_indices[i-1];
            const size_t end = flank_indices[i];
            const float dx = laser_x[end] - laser_x[start];
            const float dy = laser_y[end] - laser_y[start];
            const float dist = std::hypot(dx, dy);
            const float width = dist;  // 物体の幅

            // 幅のチェック (椅子の脚を除外)
                if(width < get_parameter("min_leg_width").as_double() || 
                width > get_parameter("max_leg_width").as_double()) {
                continue;
            }
            
            // アスペクト比チェック (円柱状の物体を優先)
            const float height = 0.7;  // 仮定の足の高さ
            const float aspect_ratio = height / width;
            if(aspect_ratio < get_parameter("leg_aspect_ratio").as_double()) {
                continue;
            }

            // 単一の足判定
            if(dist > PIERNA_DELGADA && dist < PIERNA_GRUESA) {
                legs_x.push_back((laser_x[start] + laser_x[end]) / 2);
                legs_y.push_back((laser_y[start] + laser_y[end]) / 2);
            }
            // 両足判定
            else if(dist > DOS_PIERNAS_CERCAS && dist < DOS_PIERNAS_LEJOS) {
                legs_x.push_back((laser_x[start] + laser_x[end]) / 2);
                legs_y.push_back((laser_y[start] + laser_y[end]) / 2);
            }
        }
    }

    // 足追跡処理関数
    void process_legs(const std::vector<float>& legs_x,
                     const std::vector<float>& legs_y,
                     const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        geometry_msgs::msg::PointStamped point;
        point.header = scan->header;
        point.header.frame_id = frame_id_;

        double target_x = 0.0, target_y = 0.0;  // double型で受け取る
        if(select_target_leg(legs_x, legs_y, target_x, target_y)) {
            point.point.x = target_x;
            point.point.y = target_y;
            
            apply_filter(target_x, target_y);
            point.point.x = target_x;
            point.point.y = target_y;
            
            legs_pose_pub_->publish(point);
            legs_found_ = true;
        } else {
            legs_found_ = false;
        }

        publish_marker(legs_x, legs_y);
        publish_legs_found();
    }

    // 目標足選択関数
    bool select_target_leg(const std::vector<float>& legs_x,
                         const std::vector<float>& legs_y,
                         double& x, double& y) {  // double型に変更
        if(legs_x.empty()) return false;

        // 最も近い足を選択
        size_t min_idx = 0;
        double min_dist = std::hypot(legs_x[0], legs_y[0]);
        
        for(size_t i = 1; i < legs_x.size(); ++i) {
            const double dist = std::hypot(legs_x[i], legs_y[i]);
            if(dist < min_dist) {
                min_dist = dist;
                min_idx = i;
            }
        }

        x = legs_x[min_idx];
        y = legs_y[min_idx];
        return true;
    }

    // フィルタ適用関数
    void apply_filter(double& x, double& y) {  // double型に変更
        // 入力バッファ更新
        legs_x_filter_input_.insert(legs_x_filter_input_.begin(), static_cast<float>(x));
        legs_x_filter_input_.pop_back();
        
        legs_y_filter_input_.insert(legs_y_filter_input_.begin(), static_cast<float>(y));
        legs_y_filter_input_.pop_back();

        // フィルタ係数取得
        const float bfa0x = get_parameter("bfa0x").as_double();
        const float bfa1x = get_parameter("bfa1x").as_double();
        const float bfa2x = get_parameter("bfa2x").as_double();
        const float bfb0x = get_parameter("bfb0x").as_double();
        const float bfb1x = get_parameter("bfb1x").as_double();
        const float bfb2x = get_parameter("bfb2x").as_double();

        // 完全なフィルタ計算
        float filtered_x = bfb0x * legs_x_filter_input_[0] 
                         + bfb1x * legs_x_filter_input_[1] 
                         + bfb2x * legs_x_filter_input_[2]
                         - bfa1x * legs_x_filter_output_[1] 
                         - bfa2x * legs_x_filter_output_[2];
        
        float filtered_y = bfb0x * legs_y_filter_input_[0] 
                         + bfb1x * legs_y_filter_input_[1] 
                         + bfb2x * legs_y_filter_input_[2]
                         - bfa1x * legs_y_filter_output_[1] 
                         - bfa2x * legs_y_filter_output_[2];

        // 出力バッファ更新
        legs_x_filter_output_.insert(legs_x_filter_output_.begin(), filtered_x);
        legs_x_filter_output_.pop_back();
        
        legs_y_filter_output_.insert(legs_y_filter_output_.begin(), filtered_y);
        legs_y_filter_output_.pop_back();

        // 結果を出力
        x = static_cast<double>(filtered_x);
        y = static_cast<double>(filtered_y);
    }

    // マーカーパブリッシュ関数
    void publish_marker(const std::vector<float>& legs_x,
                       const std::vector<float>& legs_y) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now();
        marker.header.frame_id = frame_id_;
        marker.ns = "leg_finder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.a = 1.0;
        marker.color.r = 0.2;
        marker.color.g = 0.6;
        marker.color.b = 0.2;

        for(size_t i = 0; i < legs_x.size(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = legs_x[i];
            p.y = legs_y[i];
            p.z = 0.3;  // 地面から30cm上に表示
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }

    // 検出状態パブリッシュ関数
    void publish_legs_found() {
        std_msgs::msg::Bool msg;
        msg.data = legs_found_;
        legs_found_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegFinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}