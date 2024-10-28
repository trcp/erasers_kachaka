k#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"
#include "kachaka_interfaces/msg/object_detection_list_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <opencv2/opencv.hpp>

class ObjectDetectionNode : public rclcpp::Node
{
public:
    ObjectDetectionNode()
        : Node("object_detection_node"),
          it_(this),
          sync_(SyncPolicy(10), image_sub_, detection_sub_)
    {
        // トピックのサブスクライバを作成
        image_sub_.subscribe(this, "front_camera/image_raw");
        detection_sub_.subscribe(this, "object_detection/result");

        // 同期ポリシーを使用して2つのトピックを同期
        sync_.registerCallback(&ObjectDetectionNode::callback, this);

        // パブリッシャを作成
        image_pub_ = it_.advertise("object_detection/result_image", 1);
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr img_msg, const kachaka_interfaces::msg::ObjectDetectionListStamped::SharedPtr detection_msg)
    {
        // OpenCVの画像に変換
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // 検出結果を画像に描画
        for (const auto& detection : detection_msg->detection)
        {
            // バウンディングボックスの情報を取得
            cv::Rect roi(detection.roi.x_offset, detection.roi.y_offset, detection.roi.width, detection.roi.height);
            cv::rectangle(cv_ptr->image, roi, cv::Scalar(0, 255, 0), 2);

            // ラベルとスコアを描画
            std::string label = std::to_string(detection.label) + ": " + std::to_string(detection.score);
            cv::putText(cv_ptr->image, label, cv::Point(detection.roi.x_offset, detection.roi.y_offset - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }

        // 描画した画像をパブリッシュ
        image_pub_.publish(cv_ptr->toImageMsg());
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        kachaka_interfaces::msg::ObjectDetectionListStamped>;

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<kachaka_interfaces::msg::ObjectDetectionListStamped> detection_sub_;
    message_filters::Synchronizer<SyncPolicy> sync_;
    image_transport::Publisher image_pub_;
    image_transport::ImageTransport it_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
}

