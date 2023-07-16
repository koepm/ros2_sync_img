#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include <std_msgs/msg/header.hpp>

using StampedImageMsg = sensor_msgs::msg::Image;
using StampedImageMsgSubscriber = message_filters::Subscriber<StampedImageMsg>;

class SimpleTimeSync : public rclcpp::Node
{
public:
  SimpleTimeSync();

private:
  std::shared_ptr<StampedImageMsgSubscriber> img1_sub_;
  std::shared_ptr<StampedImageMsgSubscriber> img2_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<StampedImageMsg, StampedImageMsg>>> approximate_sync_;
  
  void approximateSyncCallback(
    const std::shared_ptr<const sensor_msgs::msg::Image>& msg1,
    const std::shared_ptr<const sensor_msgs::msg::Image>& msg2
    );
};