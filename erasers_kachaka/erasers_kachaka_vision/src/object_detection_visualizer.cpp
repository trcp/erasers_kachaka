#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <iomanip> // for std::setprecision

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "kachaka_interfaces/msg/object_detection_list_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

// chrono_literals を使うために必要
using namespace std::chrono_literals;

class ObjectDetectionVisualizer : public rclcpp::Node
{
public:
  ObjectDetectionVisualizer()
  : Node("object_detection_visualizer")
  {
    // QoS設定
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // パブリッシャーとサブスクライバーの初期化
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("object_detection/image", qos);
    
    // Note: ユーザー提供のコードではチルダが抜けていたため補完
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "front_camera/image_raw", best_effort_qos,
      std::bind(&ObjectDetectionVisualizer::image_callback, this, std::placeholders::_1));
      
    detection_sub_ = this->create_subscription<kachaka_interfaces::msg::ObjectDetectionListStamped>(
      "object_detection/result", best_effort_qos,
      std::bind(&ObjectDetectionVisualizer::detection_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Object detection visualizer node has been started.");
  }

private:
  // 物体検出結果を保持するコールバック関数
  void detection_callback(const kachaka_interfaces::msg::ObjectDetectionListStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_detections_ = msg;
  }

  // 画像に描画を行うメインのコールバック関数
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_image;
    try {
      // ROSのImageメッセージをOpenCVで扱える形式に変換
      cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    kachaka_interfaces::msg::ObjectDetectionListStamped::ConstSharedPtr current_detections;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_detections = last_detections_;
    }
    
    // タイムアウトを1.0秒に設定
    const auto timeout = rclcpp::Duration::from_seconds(1.0);
    bool is_timed_out = !current_detections || (this->get_clock()->now() - current_detections->header.stamp > timeout);

    // === ロジックの変更箇所 ===
    // タイムアウトしておらず、かつ検出結果が1件以上ある場合のみ描画を行う
    // (注: `current_detections->detection`の`detection`はメッセージ定義のフィールド名)
    if (!is_timed_out && !current_detections->detection.empty()) {
      // 検出結果を描画する処理
      draw_detections(cv_image->image, *current_detections);
    } else {
      // タイムアウト、または検出結果が0件の場合の処理
      cv::putText(cv_image->image, "Not Detected Object's", cv::Point(20, 40), 
                  cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    // ========================
    
    // 描画した画像をパブリッシュ
    image_pub_->publish(*cv_image->toImageMsg());
  }

  // 検出結果を画像に描画するヘルパー関数
  void draw_detections(cv::Mat& image, const kachaka_interfaces::msg::ObjectDetectionListStamped& detections)
  {
    // (注: `detections.detection`の`detection`はメッセージ定義のフィールド名)
    for (const auto& detection : detections.detection) {
      // バウンディングボックスの座標を取得
      cv::Rect roi(
        detection.roi.x_offset,
        detection.roi.y_offset,
        detection.roi.width,
        detection.roi.height
      );

      // ラベルとスコアから表示テキストを作成
      std::string label_text = get_label_name(detection.label);
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << detection.score;
      label_text += " " + ss.str();

      // ラベルに応じた色を取得
      cv::Scalar color = get_label_color(detection.label);

      // バウンディングボックスを描画
      cv::rectangle(image, roi, color, 2);

      // ラベルテキストを描画
      cv::Point text_origin(roi.x, roi.y - 10 > 0 ? roi.y - 10 : roi.y + 15);
      cv::putText(image, label_text, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv::LINE_AA);
    }
  }

  // ラベルIDからラベル名を取得するヘルパー関数
  std::string get_label_name(uint8_t label)
  {
    using kachaka_interfaces::msg::ObjectDetection;
    switch (label) {
      case ObjectDetection::PERSON:  return "PERSON";
      case ObjectDetection::SHELF:   return "SHELF";
      case ObjectDetection::CHARGER: return "CHARGER";
      case ObjectDetection::DOOR:    return "DOOR";
      default:                      return "UNKNOWN";
    }
  }

  // ラベルIDから描画色を取得するヘルパー関数
  cv::Scalar get_label_color(uint8_t label)
  {
    using kachaka_interfaces::msg::ObjectDetection;
    switch (label) {
      case ObjectDetection::PERSON:  return cv::Scalar(255, 0, 0);   // Blue
      case ObjectDetection::SHELF:   return cv::Scalar(0, 255, 0);   // Green
      case ObjectDetection::CHARGER: return cv::Scalar(0, 0, 255);   // Red
      case ObjectDetection::DOOR:    return cv::Scalar(255, 255, 0); // Cyan
      default:                      return cv::Scalar(255, 255, 255); // White
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<kachaka_interfaces::msg::ObjectDetectionListStamped>::SharedPtr detection_sub_;
  
  kachaka_interfaces::msg::ObjectDetectionListStamped::ConstSharedPtr last_detections_;
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionVisualizer>());
  rclcpp::shutdown();
  return 0;
}