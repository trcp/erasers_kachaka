#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// メッセージ型
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <kachaka_interfaces/msg/object_detection_list_stamped.hpp>
#include <kachaka_interfaces/msg/object_detection.hpp>
#include <std_srvs/srv/trigger.hpp> 

#include <memory>
#include <optional>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <mutex> // mutexをインポート

using namespace std::placeholders;

struct DetectedPerson
{
  geometry_msgs::msg::Pose pose_in_map;
  int best_u; 
  double distance_from_robot;
  const kachaka_interfaces::msg::ObjectDetection* original_detection;
};


class HumanTrack : public rclcpp::Node
{
public:
  HumanTrack() : Node("human_track")
  {
    // パラメータの宣言
    this->declare_parameter<double>("tracking_distance_threshold", 1.25);
    this->declare_parameter<double>("reacquisition_timeout", 3.0);
    this->get_parameter("tracking_distance_threshold", tracking_distance_threshold_);
    this->get_parameter("reacquisition_timeout", reacquisition_timeout_);
    RCLCPP_INFO(this->get_logger(), "tracking_distance_threshold: %.2f", tracking_distance_threshold_);
    RCLCPP_INFO(this->get_logger(), "reacquisition_timeout: %.2f", reacquisition_timeout_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Publisherの初期化
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/er_kachaka/human_pose/target_pose", 10);
    all_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/er_kachaka/human_pose/poses", 10);

    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/reset_target",
      std::bind(&HumanTrack::reset_target_callback, this, _1, _2));


    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    
    sub_camera_.subscribe(this, "/er_kachaka/front_camera/image_raw", qos_profile.get_rmw_qos_profile());
    sub_camerainfo_.subscribe(this, "/er_kachaka/front_camera/camera_info", qos_profile.get_rmw_qos_profile());
    sub_lidar_.subscribe(this, "/er_kachaka/lidar/scan", qos_profile.get_rmw_qos_profile());
    sub_objectdetect_.subscribe(this, "/er_kachaka/object_detection/result", qos_profile.get_rmw_qos_profile());

    sync_ = std::make_shared<Sync>(SyncPolicy(10), sub_camera_, sub_camerainfo_, sub_lidar_, sub_objectdetect_);
    sync_->registerCallback(std::bind(&HumanTrack::sync_callback, this, _1, _2, _3, _4));
    
    RCLCPP_INFO(this->get_logger(), "Initialized Human Tracker...");
  }

private:
  void sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camerainfo,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& laserscan,
    const kachaka_interfaces::msg::ObjectDetectionListStamped::ConstSharedPtr& objectdetectionstamped)
  {
    geometry_msgs::msg::TransformStamped cam_to_laser_tf, laser_to_map_tf;
    try {
      cam_to_laser_tf = tf_buffer_->lookupTransform(
        laserscan->header.frame_id, camerainfo->header.frame_id, tf2::TimePointZero);
      laser_to_map_tf = tf_buffer_->lookupTransform(
        fixed_frame_id_, laserscan->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not look up transform: %s", ex.what());
      return;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv::Mat K = cv::Mat(3, 3, CV_64F, (void*)camerainfo->k.data());

    std::vector<DetectedPerson> current_detections;
    auto all_poses_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
    all_poses_msg->header.stamp = laserscan->header.stamp;
    all_poses_msg->header.frame_id = fixed_frame_id_;

    for (const auto& detectobject : objectdetectionstamped->detection)
    {
      if (detectobject.label != 0) continue;
      const auto& roi = detectobject.roi;
      double min_distance = std::numeric_limits<double>::max();
      int best_u = -1; 
      for (int u = roi.x_offset; u < roi.x_offset + roi.width; ++u) {
        auto current_distance = get_distance_at_pixel(u, K, laserscan, cam_to_laser_tf);
        if (current_distance && *current_distance < min_distance) {
          min_distance = *current_distance;
          best_u = u;
        }
      }
      if (best_u != -1) {
        geometry_msgs::msg::Pose pose_in_laser = get_pose_from_pixel(best_u, min_distance, K, cam_to_laser_tf);
        geometry_msgs::msg::Pose pose_in_map;
        tf2::doTransform(pose_in_laser, pose_in_map, laser_to_map_tf);
        current_detections.push_back({pose_in_map, best_u, min_distance, &detectobject});
        all_poses_msg->poses.push_back(pose_in_map);
      }
    }
    all_poses_pub_->publish(std::move(all_poses_msg));

    std::optional<DetectedPerson> current_target;
    
    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      if (is_in_cooldown_ && (this->now() - time_target_lost_).seconds() > reacquisition_timeout_) {
          RCLCPP_INFO(this->get_logger(), "Re-acquisition timeout. Finding new closest target.");
          is_in_cooldown_ = false;
          has_target_ = false;
      }

      if (has_target_ || is_in_cooldown_) {
          double closest_dist_to_last_target = std::numeric_limits<double>::max();
          std::optional<DetectedPerson> potential_target;
          for (const auto& person : current_detections) {
              double d = calculate_distance(person.pose_in_map, last_target_pose_.pose);
              if (d < closest_dist_to_last_target) {
                  closest_dist_to_last_target = d;
                  potential_target = person;
              }
          }
          if (potential_target && closest_dist_to_last_target < tracking_distance_threshold_) {
              if(is_in_cooldown_) RCLCPP_INFO(this->get_logger(), "Target re-acquired.");
              current_target = potential_target;
              has_target_ = true;
              is_in_cooldown_ = false;
          } else {
              if (has_target_) {
                RCLCPP_WARN(this->get_logger(), "Target lost! Entering re-acquisition mode for %.1f seconds.", reacquisition_timeout_);
                time_target_lost_ = this->now();
                is_in_cooldown_ = true;
              }
              has_target_ = false;
          }
      } else { 
          double min_dist_to_robot = std::numeric_limits<double>::max();
          std::optional<DetectedPerson> new_target;
          for (const auto& person : current_detections) {
              if (person.distance_from_robot < min_dist_to_robot) {
                  min_dist_to_robot = person.distance_from_robot;
                  new_target = person;
              }
          }
          if (new_target) {
              RCLCPP_INFO(this->get_logger(), "New target acquired at %.2f m.", min_dist_to_robot);
              current_target = new_target;
              has_target_ = true;
          }
      }

      if (current_target) {
          last_target_pose_.header.stamp = laserscan->header.stamp;
          last_target_pose_.header.frame_id = fixed_frame_id_;
          last_target_pose_.pose = current_target->pose_in_map;
          target_pose_pub_->publish(last_target_pose_);
      }
    }

    for (const auto& person : current_detections) {
      const auto& roi = person.original_detection->roi;
      cv::Scalar color = (current_target && person.original_detection == current_target->original_detection) ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 255, 0);
      cv::rectangle(cv_ptr->image, cv::Point(roi.x_offset, roi.y_offset), cv::Point(roi.x_offset + roi.width, roi.y_offset + roi.height), color, 2);
      std::string dist_text = cv::format("%.2f m", person.distance_from_robot);
      int center_v = roi.y_offset + roi.height / 2;
      int baseline = 0;
      cv::Size text_size = cv::getTextSize(dist_text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
      cv::rectangle(cv_ptr->image, cv::Point(person.best_u - text_size.width / 2 - 2, center_v + 15),
                    cv::Point(person.best_u + text_size.width / 2 + 2, center_v + 15 + text_size.height + baseline),
                    cv::Scalar(0, 0, 0), -1);
      cv::putText(cv_ptr->image, dist_text, cv::Point(person.best_u - text_size.width / 2, center_v + 15 + text_size.height),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
      cv::line(cv_ptr->image, cv::Point(person.best_u - 10, center_v), cv::Point(person.best_u + 10, center_v), cv::Scalar(255, 255, 0), 1);
      cv::line(cv_ptr->image, cv::Point(person.best_u, center_v - 10), cv::Point(person.best_u, center_v + 10), cv::Scalar(255, 255, 0), 1);
    }
    cv::imshow("person pose", cv_ptr->image);
    cv::waitKey(1);
  }
  
  void reset_target_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
  {
    (void)request; 

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      has_target_ = false;
      is_in_cooldown_ = false;
    }
    
    response->success = true;
    response->message = "Human tracker target has been reset.";
    RCLCPP_INFO(this->get_logger(), "Target reset via service call.");
  }

  geometry_msgs::msg::Pose get_pose_from_pixel(int u, double distance, const cv::Mat& K, const geometry_msgs::msg::TransformStamped& transform)
  {
      double cx = K.at<double>(0, 2);
      double cy = K.at<double>(1, 2);
      double fx = K.at<double>(0, 0);
      double fy = K.at<double>(1, 1);
      tf2::Vector3 ray_camera_frame((u - cx) / fx, (cy - cy) / fy, 1.0);
      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      tf2::Vector3 ray_lidar_frame = tf2::Transform(q) * ray_camera_frame;
      double target_angle_rad = std::atan2(ray_lidar_frame.y(), ray_lidar_frame.x());
      geometry_msgs::msg::Pose pose;
      pose.position.x = distance * std::cos(target_angle_rad);
      pose.position.y = distance * std::sin(target_angle_rad);
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      return pose;
  }
  
  double calculate_distance(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2) {
    return std::hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
  }

  std::optional<double> get_distance_at_pixel(
    int u, const cv::Mat& K,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& laserscan,
    const geometry_msgs::msg::TransformStamped& transform)
  {
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    tf2::Vector3 ray_camera_frame((u - cx) / fx, (cy - cy) / fy, 1.0);
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    tf2::Vector3 ray_lidar_frame = tf2::Transform(q) * ray_camera_frame;
    double target_angle_rad = std::atan2(ray_lidar_frame.y(), ray_lidar_frame.x());
    if (laserscan->angle_increment == 0.0) return std::nullopt;
    double min_angle_diff = std::numeric_limits<double>::max();
    int best_index = -1;
    for (size_t i = 0; i < laserscan->ranges.size(); ++i) {
      double current_angle = laserscan->angle_min + i * laserscan->angle_increment;
      double angle_diff = std::abs(std::atan2(std::sin(current_angle - target_angle_rad), std::cos(current_angle - target_angle_rad)));
      if (angle_diff < min_angle_diff) {
        min_angle_diff = angle_diff;
        best_index = i;
      }
    }
    if (best_index != -1) {
      double distance = laserscan->ranges[best_index];
      if (std::isinf(distance) || std::isnan(distance) || distance < laserscan->range_min || distance > laserscan->range_max) {
        return std::nullopt;
      }
      return distance;
    }
    return std::nullopt;
  }
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_poses_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_camera_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camerainfo_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_lidar_;
  message_filters::Subscriber<kachaka_interfaces::msg::ObjectDetectionListStamped> sub_objectdetect_;
  
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, kachaka_interfaces::msg::ObjectDetectionListStamped>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::string fixed_frame_id_ = "map";

  // 追跡用メンバ変数
  bool has_target_ = false;
  bool is_in_cooldown_ = false;
  geometry_msgs::msg::PoseStamped last_target_pose_;
  rclcpp::Time time_target_lost_;
  std::mutex state_mutex_;

  double tracking_distance_threshold_;
  double reacquisition_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanTrack>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}