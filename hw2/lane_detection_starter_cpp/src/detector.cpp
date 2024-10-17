#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#ifdef ROS_DISTRO_JAZZ
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "lane_detection_msgs/msg/detection_target.hpp"
#include "lane_detection_msgs/msg/detection_result.hpp"
#include "lane_detection_msgs/msg/lane.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Detector : public rclcpp::Node
{
public:
  Detector() : Node("detector")
  {
    RCLCPP_INFO(this->get_logger(), "Detector node has been started");

    result_pub_ = this->create_publisher<lane_detection_msgs::msg::DetectionResult>(
      "/detection_result", 10);
    detected_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detected_image", 10);

    target_sub_ = this->create_subscription<lane_detection_msgs::msg::DetectionTarget>(
      "/detection_target", 10, std::bind(&Detector::target_callback, this, _1));
  }

private:
  void target_callback(const lane_detection_msgs::msg::DetectionTarget::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received detection target with frame_id %d", msg->frame_id);

    auto start_time = std::chrono::high_resolution_clock::now();

    lane_detection_msgs::msg::DetectionResult detection_result;
    detection_result.header = msg->header;
    detection_result.frame_id = msg->frame_id;
    detection_result.gt_lanes = msg->gt_lanes;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg->image_raw, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    auto detected_lanes = lane_detection(cv_ptr->image);

    for (const auto& lane : detected_lanes) {
        lane_detection_msgs::msg::Lane lane_msg;
        lane_msg.coordinates.assign(lane.begin(), lane.end());
        detection_result.lanes.push_back(lane_msg);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    detection_result.run_time = duration.count();

    RCLCPP_INFO(this->get_logger(), "Detected %zu lanes with run time %.2f ms", 
                detected_lanes.size(), detection_result.run_time);

    std::vector<std::vector<uint32_t>> gt_lanes;
    for (const auto& lane : msg->gt_lanes) {
        gt_lanes.push_back(lane.coordinates);
    }

    draw_lanes(cv_ptr->image, detected_lanes, gt_lanes);

    result_pub_->publish(detection_result);
    detected_img_pub_->publish(*(cv_ptr->toImageMsg()));
  }

   cv::Mat canny_edge_detection(const cv::Mat& img_cv)
    {
        cv::Mat gray, blur, canny;
        cv::cvtColor(img_cv, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
        cv::Canny(blur, canny, 50, 150);
        return canny;
    }

    cv::Mat region_of_interest(const cv::Mat& canny)
    {
        cv::Mat mask = cv::Mat::zeros(canny.size(), canny.type());
        cv::Point pts[1][3];
        pts[0][0] = cv::Point(200, canny.rows);
        pts[0][1] = cv::Point(1100, canny.rows);
        pts[0][2] = cv::Point(550, 250);

        const cv::Point* ppt[1] = {pts[0]};
        int npt[] = {3};

        cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255, 255, 255), cv::LINE_8);
        cv::Mat masked_image;
        cv::bitwise_and(canny, mask, masked_image);
        return masked_image;
    }

    std::vector<std::vector<uint32_t>> lane_detection(const cv::Mat& img_cv)
    {
        cv::Mat canny = canny_edge_detection(img_cv);
        cv::Mat cropped_image = region_of_interest(canny);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(cropped_image, lines, 2, CV_PI / 180, 100, 40, 5);

        std::vector<std::vector<uint32_t>> return_lines;
        for (const auto& line : lines)
        {
            return_lines.push_back({line[0], line[1], line[2], line[3]});
        }
        return return_lines;
    }

    void draw_lanes(cv::Mat& img_cv, const std::vector<std::vector<uint32_t>>& lanes, const std::vector<std::vector<uint32_t>>& gt_lanes)
    {
        for (const auto& lane : lanes)
        {
            for (size_t i = 0; i < lane.size(); i += 2)
            {
                if (i + 1 < lane.size())
                {
                    int x = lane[i];
                    int y = lane[i + 1];
                    cv::circle(img_cv, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
                }
            }
        }

        for (const auto& lane : gt_lanes)
        {
            for (size_t i = 0; i < lane.size(); i += 2)
            {
                if (i + 1 < lane.size())
                {
                    int x = lane[i];
                    int y = lane[i + 1];
                    cv::circle(img_cv, cv::Point(x, y), 5, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
    }


  rclcpp::Publisher<lane_detection_msgs::msg::DetectionResult>::SharedPtr result_pub_;
  rclcpp::Subscription<lane_detection_msgs::msg::DetectionTarget>::SharedPtr target_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_img_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Detector>());
  rclcpp::shutdown();
  return 0;
}
