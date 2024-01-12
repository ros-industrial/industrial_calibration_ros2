#include "industrial_calibration_ros/target_detector.h"

const std::string IMAGE_RAW_TOPIC = "target_detector/image_raw";
const std::string IMAGE_DETECTED_TOPIC = "target_detector/image_detected";

TargetDetector::TargetDetector(ros::NodeHandle& nh) : nh_(nh), it_(nh)
{ 
  // Load the target finder
  std::string config_path;
  bool paramReceived = nh_.getParam("target_detector_config_path", config_path);
  YAML::Node config = YAML::LoadFile(config_path);
  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
  loader.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;

  YAML::Node target_finder_config = getMember<YAML::Node>(config, "target_finder");
  auto factory = loader.createInstance<industrial_calibration::TargetFinderFactory>(getMember<std::string>(target_finder_config, "type"));
  target_finder_ = factory->create(target_finder_config);

  // Setup subscriber and publishers
  nh.getParam("target_detector/image_sub_topic", imageSubTopic_);
  imageSub_ = it_.subscribe(imageSubTopic_, 1, &TargetDetector::imageCb, this);
  imageRawPub_ = it_.advertise(IMAGE_RAW_TOPIC, 1);
  detectionPub_ = it_.advertise(IMAGE_DETECTED_TOPIC, 1);
}

void TargetDetector::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // Convert to CvImagePtr
    if (msg->encoding == "mono16")
    {
      cv_bridge::CvImagePtr temp_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);

      cv::Mat img_conv;
      cv::cvtColor(temp_ptr->image, img_conv, cv::COLOR_GRAY2BGR);
      img_conv.convertTo(img_conv, CV_8UC1);
      cv_ptr = cv_bridge::CvImagePtr(
          new cv_bridge::CvImage(temp_ptr->header, sensor_msgs::image_encodings::BGR8, img_conv));
    }
    else
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    // Find target in image
    cv::Mat tmp_image = cv_ptr->image.clone();
    industrial_calibration::TargetFeatures target_features = target_finder_->findTargetFeatures(tmp_image);
    cv::Mat modified_image = target_finder_->drawTargetFeatures(tmp_image, target_features);
    cv_bridge::CvImagePtr detection_ptr(new cv_bridge::CvImage(cv_ptr->header, cv_ptr->encoding, modified_image));

    // Publish raw_image and image with drawn features
    imageRawPub_.publish(cv_ptr->toImageMsg());
    detectionPub_.publish(detection_ptr->toImageMsg());
  }
  catch (const std::runtime_error& ex)
  {
    ROS_ERROR_STREAM(ex.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_detector_node");
  ros::NodeHandle nh;
  TargetDetector targetDetectorNode(nh);
  ros::spin();
  
  ros::shutdown();
  return 0;
}