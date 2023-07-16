#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "sync_msg/sync_img.hpp"

SimpleTimeSync::SimpleTimeSync()
: Node("img_sync")
{
  img1_sub_ = std::make_shared<StampedImageMsgSubscriber>(this, "video1", rmw_qos_profile_sensor_data);
  img2_sub_ = std::make_shared<StampedImageMsgSubscriber>(this, "video2", rmw_qos_profile_sensor_data);

  approximate_sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<StampedImageMsg, StampedImageMsg>>>(
    message_filters::sync_policies::ApproximateTime<StampedImageMsg, StampedImageMsg>(10), *img1_sub_, *img2_sub_);

  approximate_sync_->registerCallback(std::bind(&SimpleTimeSync::approximateSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void SimpleTimeSync::approximateSyncCallback(
  const std::shared_ptr<const sensor_msgs::msg::Image>& msg1,
  const std::shared_ptr<const sensor_msgs::msg::Image>& msg2)
{
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_bridge::CvImagePtr cv_ptr_right;

    cv_ptr_left = cv_bridge::toCvCopy(msg1, "bgr8");
    cv_ptr_right = cv_bridge::toCvCopy(msg2, "bgr8");
    cv::imshow("left", cv_ptr_left->image);
    cv::imshow("right", cv_ptr_right->image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleTimeSync>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
