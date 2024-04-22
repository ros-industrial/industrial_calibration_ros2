#include "utils.h"

#include <boost_plugin_loader/plugin_loader.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <industrial_calibration/core/serialization.h>
#include <industrial_calibration/target_finders/opencv/target_finder.h>
#include <industrial_calibration/optimizations/pnp.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>

#include <fstream>

template<typename T>
T getParameter(ros::NodeHandle& nh, const std::string& key)
{
  T val;
  if(!nh.getParam(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

class PnPValidator
{
public:
  PnPValidator()
    : pnh_("~")
    , it_(ros::NodeHandle())
    , image_sub_(it_.subscribe("image", 1, &PnPValidator::imageCb, this))
    , last_image_(nullptr)
    , validate_server_(nh_.advertiseService("validate_pnp", &PnPValidator::validatePnP, this))
    , compute_server_(nh_.advertiseService("compute_pnp", &PnPValidator::computePnP, this))
  {
    // Configure the plugin loader
    loader_.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
    loader_.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;

    // Load the target finder
    YAML::Node config = YAML::LoadFile(getParameter<std::string>(pnh_, "config_file"));

    // Load the target finder
    YAML::Node target_finder_config = getMember<YAML::Node>(config, "target_finder");
    factory_ = loader_.createInstance<industrial_calibration::TargetFinderFactoryOpenCV>(getMember<std::string>(target_finder_config, "type"));
    target_finder_ = factory_->create(target_finder_config);

    // Load the camera intrinsics
    pnp_.intr = getMember<industrial_calibration::CameraIntrinsics>(config, "intrinsics");

    // Load the known pose of the target
    pnp_.camera_to_target_guess = getMember<Eigen::Isometry3d>(config, "camera_to_target");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    last_image_ = msg;
  }

  void checkImage()
  {
    if(!last_image_)
      throw std::runtime_error("No image yet acquired");

    // Check the time of the last image
    double time_tolerance = getParameter<double>(pnh_, "time_tolerance");
    double time_delta = (ros::Time::now() - last_image_->header.stamp).toSec();
    if (time_delta > time_tolerance)
    {
      std::stringstream ss;
      ss << "Last acquired image is " << time_delta - time_tolerance << " seconds too old";
      throw std::runtime_error(ss.str());
    }
  }

  bool computePnP(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    try
    {
      checkImage();

      // Convert the image message to OpenCV
      cv_bridge::CvImagePtr cv_ptr = fromROS(last_image_);

      // Find target in image and replace the correspondences
      pnp_.correspondences = target_finder_->findCorrespondences(cv_ptr->image);

      // Perform the PnP optimization
      industrial_calibration::PnPResult pnp_result = industrial_calibration::optimize(pnp_);
      if(!pnp_result.converged)
        throw std::runtime_error("PnP optimization did not converge");

      // Save to file
      YAML::Node node(pnp_result.camera_to_target);
      {
        std::ofstream f("/tmp/pnp.yaml");
        f << node;
      }

      res.success = true;
      std::stringstream ss;
      ss << node;
      res.message = ss.str();
    }
    catch(const std::exception& ex)
    {
      res.success = false;
      res.message = ex.what();
    }

    return true;
  }

  bool validatePnP(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    try
    {
      checkImage();

      // Convert the image message to OpenCV
      cv_bridge::CvImagePtr cv_ptr = fromROS(last_image_);

      // Find target in image and replace the correspondences
      pnp_.correspondences = target_finder_->findCorrespondences(cv_ptr->image);

      // Perform the PnP optimization
      industrial_calibration::PnPResult pnp_result = industrial_calibration::optimize(pnp_);
      if(!pnp_result.converged)
        throw std::runtime_error("PnP optimization did not converge");

      // Compute the pose difference
      Eigen::Isometry3d diff = pnp_.camera_to_target_guess.inverse() * pnp_result.camera_to_target;

      // Compare pose difference to tolerance
      double position_diff = diff.translation().norm();
      double position_tolerance = getParameter<double>(pnh_, "position_tolerance");
      if(position_diff > position_tolerance)
      {
        std::stringstream ss;
        ss << "Translation difference exceeds tolerance by " << position_diff - position_tolerance << " (m)";
        throw std::runtime_error(ss.str());
      }

      double orientation_diff = Eigen::Quaterniond::Identity().angularDistance(Eigen::Quaterniond(diff.linear()));
      double orientation_tolerance = getParameter<double>(pnh_, "orientation_tolerance");
      if(orientation_diff > orientation_tolerance)
      {
        std::stringstream ss;
        ss << "Orientation difference exceeds tolerance by " << orientation_diff - orientation_tolerance << " (m)";
        throw std::runtime_error(ss.str());
      }

      res.success = true;
      std::stringstream ss;
      ss << "Target is within tolerance (" << position_diff << " (m), " << orientation_tolerance * 180.0 / M_PI << " (deg))";
      res.message = ss.str();
    }
    catch (const std::runtime_error& ex)
    {
      res.success = false;
      res.message = ex.what();
    }

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::Image::ConstPtr last_image_;
  ros::ServiceServer compute_server_;
  ros::ServiceServer validate_server_;

  boost_plugin_loader::PluginLoader loader_;
  industrial_calibration::TargetFinderFactoryOpenCV::Ptr factory_;
  industrial_calibration::TargetFinderOpenCV::ConstPtr target_finder_;
  industrial_calibration::PnPProblem pnp_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pnp_validation_node");
  PnPValidator validator;
  ROS_INFO_STREAM("Started PnP validation node...");
  ros::spin();
  return 0;
}
