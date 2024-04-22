#pragma once

#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

cv_bridge::CvImagePtr fromROS(sensor_msgs::ImageConstPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  // Check the bit-depth
  if(sensor_msgs::image_encodings::bitDepth(msg->encoding) == 8)
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  else
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv::normalize(cv_ptr->image, cv_ptr->image, 0, 255, cv::NormTypes::NORM_MINMAX);
    cv_ptr->image.convertTo(cv_ptr->image, CV_8U);
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
  }

  // Check the number of channels
  if(cv_ptr->image.channels() != 3)
    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2BGR);

  return cv_ptr;
}
