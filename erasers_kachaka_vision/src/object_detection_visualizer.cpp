#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <algorithm> // for std::sort
#include <cmath> // for std::isfinite

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "kachaka_interfaces/msg/object_detection_list_stamped.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "image_geometry/pinhole_camera_model.h"

using namespace std::chrono_literals;

class ObjectDetectionVisualizer : public rclcpp::Node
{
public:
  explicit ObjectDetectionVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("object_detection_visualizer", options)
  {
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
      RCLCPP_INFO(this->get_logger(), "Mode: Simulation Time (Reading from Rosbag or Simulator)");
    } else {
      RCLCPP_INFO(this->get_logger(), "Mode: Realtime (System Clock)");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    rmw_qos_profile_t best_effort_rmw_qos = best_effort_qos.get_rmw_qos_profile();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("object_detection/image", qos);
    
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("object_detection/markers", qos);

    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("object_detection/poses", qos);

    rgb_image_sub_.subscribe(this, "front_camera/reliable/image_raw", best_effort_rmw_qos);
    rgb_info_sub_.subscribe(this, "front_camera/reliable/camera_info", best_effort_rmw_qos);
    depth_image_sub_.subscribe(this, "tof_camera/registered/image_rect", best_effort_rmw_qos);
    detection_sub_.subscribe(this, "object_detection/result", best_effort_rmw_qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(30),
      rgb_image_sub_,
      rgb_info_sub_,
      depth_image_sub_,
      detection_sub_
    );

    sync_->registerCallback(std::bind(
      &ObjectDetectionVisualizer::sync_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4
    ));

    RCLCPP_INFO(this->get_logger(), "Object detection visualizer node has been started.");
  }

private:
  void sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const kachaka_interfaces::msg::ObjectDetectionListStamped::ConstSharedPtr& detection_msg)
  {
    cv_bridge::CvImagePtr cv_image;
    cv_bridge::CvImagePtr depth_cv_image;

    try {
      cv_image = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
      depth_cv_image = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(info_msg);

    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::PoseArray pose_array;

    pose_array.header = info_msg->header;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header = info_msg->header;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int marker_id = 0; 

    if (!detection_msg->detection.empty()) {
      
      draw_detections(cv_image->image, *detection_msg);

      for (const auto& detection : detection_msg->detection) {
        cv::Rect roi(
          detection.roi.x_offset,
          detection.roi.y_offset,
          detection.roi.width,
          detection.roi.height
        );

        float median_depth = get_median_depth(depth_cv_image, roi);

        if (median_depth <= 0.0) {
          continue; 
        }

        cv::Point2d center_pixel(roi.x + roi.width / 2.0, roi.y + roi.height / 2.0);
        cv::Point3d ray_center = cam_model.projectPixelTo3dRay(center_pixel);
        cv::Point3d pos_center = ray_center * median_depth;

        geometry_msgs::msg::Pose pose; // PoseArray と Marker で共有
        pose.position.x = pos_center.x;
        pose.position.y = pos_center.y;
        pose.position.z = pos_center.z;
        pose.orientation.w = 1.0;

        pose_array.poses.push_back(pose);

        visualization_msgs::msg::Marker cube_marker;
        cube_marker.header = info_msg->header; 
        cube_marker.ns = "detection_cubes";
        cube_marker.id = marker_id++;
        cube_marker.type = visualization_msgs::msg::Marker::CUBE;
        cube_marker.action = visualization_msgs::msg::Marker::ADD;
        
        cube_marker.pose = pose; 

        cv::Point2d top_left_pixel(roi.x, roi.y);
        cv::Point2d bottom_right_pixel(roi.x + roi.width, roi.y + roi.height);
        cv::Point3d ray_tl = cam_model.projectPixelTo3dRay(top_left_pixel);
        cv::Point3d ray_br = cam_model.projectPixelTo3dRay(bottom_right_pixel);
        cv::Point3d pos_tl = ray_tl * median_depth;
        cv::Point3d pos_br = ray_br * median_depth;

        double scale_x = std::abs(pos_br.x - pos_tl.x);
        double scale_y = std::abs(pos_br.y - pos_tl.y);
        double scale_z = (scale_x + scale_y) / 2.0;

        cube_marker.scale.x = std::max(0.01, scale_x);
        cube_marker.scale.y = std::max(0.01, scale_y);
        cube_marker.scale.z = std::max(0.01, scale_z);

        cv::Scalar cv_color = get_label_color(detection.label);
        cube_marker.color.a = 0.5;
        cube_marker.color.r = cv_color[2] / 255.0;
        cube_marker.color.g = cv_color[1] / 255.0;
        cube_marker.color.b = cv_color[0] / 255.0;

        cube_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        marker_array.markers.push_back(cube_marker);


        visualization_msgs::msg::Marker text_marker;
        text_marker.header = info_msg->header;
        text_marker.ns = "detection_labels";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.text = get_label_name(detection.label);
        
        text_marker.pose.position.x = pos_center.x;
        text_marker.pose.position.y = pos_center.y;
        text_marker.pose.position.z = pos_center.z + (scale_z / 2.0) + 0.1;
        
        text_marker.scale.z = 0.1;
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        
        text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        marker_array.markers.push_back(text_marker);
      }

    } else {
      cv::putText(cv_image->image, "Not Detected Object's", cv::Point(20, 40), 
                  cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    
    image_pub_->publish(*cv_image->toImageMsg());
    marker_array_pub_->publish(marker_array);
    
    if (!pose_array.poses.empty()) {
      pose_array_pub_->publish(pose_array);
    }
  }

  float get_median_depth(
    const cv_bridge::CvImagePtr& depth_image, 
    const cv::Rect& roi)
  {
    std::vector<float> depths;

    cv::Rect valid_roi = roi & cv::Rect(0, 0, depth_image->image.cols, depth_image->image.rows);
    if (valid_roi.area() == 0) {
      return 0.0;
    }

    if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      for (int v = valid_roi.y; v < valid_roi.y + valid_roi.height; ++v) {
        for (int u = valid_roi.x; u < valid_roi.x + valid_roi.width; ++u) {
          float depth = depth_image->image.at<float>(v, u);
          if (depth > 0.01 && depth < 10.0 && std::isfinite(depth)) { 
            depths.push_back(depth);
          }
        }
      }
    } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      for (int v = valid_roi.y; v < valid_roi.y + valid_roi.height; ++v) {
        for (int u = valid_roi.x; u < valid_roi.x + valid_roi.width; ++u) {
          uint16_t depth_mm = depth_image->image.at<uint16_t>(v, u);
          if (depth_mm > 10 && depth_mm < 10000) { 
            depths.push_back(static_cast<float>(depth_mm) / 1000.0f);
          }
        }
      }
    } else {
      RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported depth encoding: %s", depth_image->encoding.c_str());
      return 0.0;
    }

    if (depths.empty()) {
      return 0.0;
    }

    std::sort(depths.begin(), depths.end());
    float median_depth = depths[depths.size() / 2];
    
    return median_depth;
  }

  void draw_detections(cv::Mat& image, const kachaka_interfaces::msg::ObjectDetectionListStamped& detections)
  {
    for (const auto& detection : detections.detection) {
      cv::Rect roi(
        detection.roi.x_offset,
        detection.roi.y_offset,
        detection.roi.width,
        detection.roi.height
      );
      std::string label_text = get_label_name(detection.label);
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << detection.score;
      label_text += " " + ss.str();
      cv::Scalar color = get_label_color(detection.label);
      cv::rectangle(image, roi, color, 2);
      cv::Point text_origin(roi.x, roi.y - 10 > 0 ? roi.y - 10 : roi.y + 15);
      cv::putText(image, label_text, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv::LINE_AA);
    }
  }

  std::string get_label_name(uint8_t label)
  {
    using kachaka_interfaces::msg::ObjectDetection;
    switch (label) {
      case ObjectDetection::PERSON:  return "PERSON";
      case ObjectDetection::SHELF:   return "SHELF";
      case ObjectDetection::CHARGER: return "CHARGER";
      case ObjectDetection::DOOR:    return "DOOR";
      default:                       return "UNKNOWN";
    }
  }

  cv::Scalar get_label_color(uint8_t label)
  {
    using kachaka_interfaces::msg::ObjectDetection;
    switch (label) {
      case ObjectDetection::PERSON:  return cv::Scalar(255, 0, 0);
      case ObjectDetection::SHELF:   return cv::Scalar(0, 255, 0);
      case ObjectDetection::CHARGER: return cv::Scalar(0, 0, 255);
      case ObjectDetection::DOOR:    return cv::Scalar(255, 255, 0);
      default:                       return cv::Scalar(255, 255, 255);
    }
  }

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image,
    kachaka_interfaces::msg::ObjectDetectionListStamped
  > SyncPolicy;
  
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> rgb_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
  message_filters::Subscriber<kachaka_interfaces::msg::ObjectDetectionListStamped> detection_sub_;
  
  std::shared_ptr<Sync> sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionVisualizer>());
  rclcpp::shutdown();
  return 0;
}
