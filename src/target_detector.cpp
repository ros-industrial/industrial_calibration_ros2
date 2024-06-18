#include "utils.h"

#include <boost_plugin_loader/plugin_loader.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <industrial_calibration/core/serialization.h>
#include <industrial_calibration/target_finders/opencv/target_finder.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

template <typename T>
T getParameter(rclcpp::Node::SharedPtr node, const std::string& key)
{
  T val;
  if (!node->get_parameter(key, val)) throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

class TargetDetector
{
public:
  TargetDetector(std::shared_ptr<rclcpp::Node> node) : node_(node), it_(node_)
  {
    // Configure the plugin loader
    loader_.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
    loader_.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;

    // Load the target finder
    node_->declare_parameter("config_file", "");
    YAML::Node config = YAML::LoadFile(getParameter<std::string>(node_, "config_file"));
    YAML::Node target_finder_config = getMember<YAML::Node>(config, "target_finder");
    factory_ = loader_.createInstance<industrial_calibration::TargetFinderFactoryOpenCV>(
        getMember<std::string>(target_finder_config, "type"));
    target_finder_ = factory_->create(target_finder_config);

    // Setup subscriber and publishers
    image_sub_ = it_.subscribe("image", 1, std::bind(&TargetDetector::imageCb, this, std::placeholders::_1));

    detected_image_pub_ = it_.advertise("image_detected", 1);
    annotated_image_pub_ = it_.advertise("image_annotated", 1);
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr = fromROS(msg);

      // Find target in image
      industrial_calibration::TargetFeatures2D target_features = target_finder_->findTargetFeatures(cv_ptr->image);
      cv::Mat annotated_image = target_finder_->drawTargetFeatures(cv_ptr->image, target_features);
      cv_bridge::CvImagePtr annotated_image_cv(
          new cv_bridge::CvImage(cv_ptr->header, cv_ptr->encoding, annotated_image));

      // Publish raw_image and image with drawn features
      detected_image_pub_.publish(msg);
      annotated_image_pub_.publish(annotated_image_cv->toImageMsg());
    }
    catch (const std::runtime_error& ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_;

  boost_plugin_loader::PluginLoader loader_;
  industrial_calibration::TargetFinderFactoryOpenCV::Ptr factory_;
  industrial_calibration::TargetFinderOpenCV::ConstPtr target_finder_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher detected_image_pub_;
  image_transport::Publisher annotated_image_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("target_detector_node");
  auto target_detector = std::make_shared<TargetDetector>(node);
  RCLCPP_INFO(node->get_logger(), "Started target detector node...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
