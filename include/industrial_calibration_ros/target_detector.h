#ifndef TARGET_DETECTOR_HPP
#define TARGET_DETECTOR_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <yaml-cpp/yaml.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <string>
#include <industrial_calibration/target_finders/target_finder.h>
#include <industrial_calibration/serialization.h>

class TargetDetector
{
  public:
    TargetDetector(ros::NodeHandle& nh);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    std::string imageSubTopic_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imageRawPub_;
    image_transport::Publisher detectionPub_;
    industrial_calibration::TargetFinder::ConstPtr target_finder_;
};

#endif