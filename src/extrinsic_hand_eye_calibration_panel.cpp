#include <industrial_calibration/gui/extrinsic_hand_eye_calibration_widget.h>
#include <industrial_calibration/gui/transform_guess.h>
#include <industrial_calibration/gui/aspect_ratio_pixmap_label.h>
#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>
#include <QAction>
#include <QLCDNumber>
#include <QMessageBox>
#include <QTextStream>
#include <QToolBar>
#include <QVBoxLayout>
#include <QSplitter>
#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <std_srvs/srv/trigger.hpp>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

static const char* CAMERA_MOUNT_FRAME_PARAM = "camera_mount_frame";
static const char* CAMERA_FRAME_PARAM = "camera_frame";
static const char* TARGET_MOUNT_FRAME_PARAM = "target_mount_frame";
static const char* TARGET_FRAME_PARAM = "target_frame";

namespace industrial_calibration_ros
{
class ExtrinsicHandEyeCalibrationPanel : public rviz_common::Panel
{
public:
  void onInitialize() override;

protected:
  template <typename T>
  bool get_parameter(const std::string& key, T& val)
  {
    const bool ret = node_->get_parameter<T>(key, val);
    if (!ret)
    {
      QString message;
      QTextStream stream(&message);
      stream << "Failed to get parameter '" << QString::fromStdString(key) << "'";
      QMessageBox::warning(this, "Parameter Error", message);
    }
    return ret;
  }

  void populateTransformGuess(const std::string& from_frame_param, const std::string& to_frame_param,
                              industrial_calibration::TransformGuess* widget);

  void callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                   rclcpp::Client<std_srvs::srv::Trigger>::CallbackType);

  void triggerObservationCallback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
  void saveCallback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  void onCalibrationComplete(const industrial_calibration::ExtrinsicHandEyeResult& result);

  QLCDNumber* observation_counter_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_trigger_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_save_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

template <>
bool ExtrinsicHandEyeCalibrationPanel::get_parameter<std::string>(const std::string& key, std::string& val)
{
  bool ret = node_->get_parameter<std::string>(key, val);
  if (!ret)
  {
    QString message;
    QTextStream stream(&message);
    stream << "Failed to get parameter '" << QString::fromStdString(key) << "'";
    QMessageBox::warning(this, "Parameter Error", message);
  }

  if (val.empty())
  {
    QString message;
    QTextStream stream(&message);
    stream << "Parameter '" << QString::fromStdString(key) << "' is empty";
    QMessageBox::warning(this, "Error", message);
    ret = false;
  }

  return ret;
}

void ExtrinsicHandEyeCalibrationPanel::onInitialize()
{
  // Initialize the ROS interfaces
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  client_trigger_ = node_->create_client<std_srvs::srv::Trigger>("trigger");
  client_save_ = node_->create_client<std_srvs::srv::Trigger>("save");
  buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock(), std::chrono::seconds(3));
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_, false);
  broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  auto layout = new QVBoxLayout(this);

  // Add an instructions action to the tool bar
  auto action_instructions = new QAction("Instructions", this);
  action_instructions->setToolTip("Click for instructions on collecting calibration data");
  action_instructions->setIcon(QIcon::fromTheme("dialog-information"));
  connect(action_instructions, &QAction::triggered, []() {
    const std::string instructions = R"(
<html>
<body>
<h3>Extrinsic Hand Eye Calibration Data Collection</h3>
<p>
  <ol>
    <li>
      Run the 2D data collection launch file.
      <pre><code>ros2 launch data_collection.launch.xml</code></pre>
      Make sure to specify the 2D image topic and relevant calibration frames as arguments to the launch file.
      Ensure TF is running and can look up transforms between the specified calibration frames.
    </li>
    <li>
      Acquire observations of the calibration target from multiple view points (10-15 is generally sufficient).
    </li>
    <li>
      Once enough observations have been collected, save them to file.
    </li>
    <li>
      Follow the instructions provided by the information button in the calibration tool bar to perform the calibration.
    </li>
  </ol>
</p>
</body>
</html>
)";
    QMessageBox::information(nullptr, "Instructions", QString::fromStdString(instructions));
  });

  // Add an LCD number to count the number of successful observation triggers
  observation_counter_ = new QLCDNumber(this);
  observation_counter_->setDigitCount(2);
  observation_counter_->setToolTip("Number of observations collected");

  // Add an action for calling the trigger service
  auto action_trigger = new QAction("Trigger observation collection", this);
  action_trigger->setToolTip("Calls the observation trigger service using ROS");
  action_trigger->setIcon(QIcon::fromTheme("insert-image"));
  connect(action_trigger, &QAction::triggered, [this](const bool) {
    auto cb = std::bind(&ExtrinsicHandEyeCalibrationPanel::triggerObservationCallback, this, std::placeholders::_1);
    callService(client_trigger_, cb);
  });

  // Add an action for calling the save service
  auto action_save = new QAction("Save calibration data", this);
  action_save->setToolTip("Calls the calibration data save service using ROS");
  action_save->setIcon(QIcon::fromTheme("document-save-as"));
  connect(action_save, &QAction::triggered, [this](const bool) {
    auto cb = std::bind(&ExtrinsicHandEyeCalibrationPanel::saveCallback, this, std::placeholders::_1);
    callService(client_save_, cb);
  });

  // Add a tool bar for the ROS functionality
  {
    auto tool_bar = new QToolBar(this);
    tool_bar->addAction(action_instructions);
    tool_bar->addSeparator();
    tool_bar->addAction(action_trigger);
    tool_bar->addWidget(observation_counter_);
    tool_bar->addSeparator();
    tool_bar->addAction(action_save);
    layout->addWidget(tool_bar);
  }

  // Add the extrinsic hand eye calibration widget
  auto cal = new industrial_calibration::ExtrinsicHandEyeCalibrationWidget(this);

  // Add the calibration tool bar to the layout
  layout->addWidget(cal->tool_bar);

  // Add an image viewer for the selected calibration image
  auto image = new industrial_calibration::AspectRatioPixmapLabel(this);
  image->setAlignment(Qt::AlignCenter);
  connect(cal, &industrial_calibration::ExtrinsicHandEyeCalibrationWidget::imageSelected, image,
          &industrial_calibration::AspectRatioPixmapLabel::setPixmap);

  auto splitter = new QSplitter(Qt::Orientation::Vertical, this);
  splitter->addWidget(cal);
  splitter->addWidget(image);
  layout->addWidget(splitter);

  // Declare parameters for looking up guess transforms
  node_->declare_parameter<std::string>(CAMERA_MOUNT_FRAME_PARAM, "");
  node_->declare_parameter<std::string>(CAMERA_FRAME_PARAM, "");
  node_->declare_parameter<std::string>(TARGET_MOUNT_FRAME_PARAM, "");
  node_->declare_parameter<std::string>(TARGET_FRAME_PARAM, "");

  // Connect the guess transform actions to a callback that can populate its values from TF
  connect(cal->action_camera_mount_to_camera, &QAction::triggered, [=]() {
    populateTransformGuess(CAMERA_MOUNT_FRAME_PARAM, CAMERA_FRAME_PARAM, cal->camera_transform_guess_widget_);
  });

  connect(cal->action_target_mount_to_target, &QAction::triggered, [=]() {
    populateTransformGuess(TARGET_MOUNT_FRAME_PARAM, TARGET_FRAME_PARAM, cal->target_transform_guess_widget_);
  });

  // Connect the calibration complete signal to a callback that will publish the calibrated frames to TF
  connect(cal, &industrial_calibration::ExtrinsicHandEyeCalibrationWidget::calibrationComplete, this,
          &ExtrinsicHandEyeCalibrationPanel::onCalibrationComplete);
}

void ExtrinsicHandEyeCalibrationPanel::populateTransformGuess(const std::string& from_frame_param,
                                                              const std::string& to_frame_param,
                                                              industrial_calibration::TransformGuess* widget)
{
  switch (QMessageBox::question(this, "TF", "Use TF to populate transform guess?"))
  {
    case QMessageBox::StandardButton::No:
      return;
    default:
      break;
  }

  std::string from_frame;
  if (!get_parameter<std::string>(from_frame_param, from_frame)) return;

  std::string to_frame;
  if (!get_parameter<std::string>(to_frame_param, to_frame)) return;

  std::string message;
  if (!buffer_->canTransform(from_frame, to_frame, tf2::TimePointZero, &message))
  {
    QMessageBox::warning(this, "TF Error", QString::fromStdString(message));
    return;
  }

  geometry_msgs::msg::TransformStamped transform = buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);

  // Configure the widget with the resulting transform
  YAML::Node node;
  node["x"] = transform.transform.translation.x;
  node["y"] = transform.transform.translation.y;
  node["z"] = transform.transform.translation.z;
  node["qx"] = transform.transform.rotation.x;
  node["qy"] = transform.transform.rotation.y;
  node["qz"] = transform.transform.rotation.z;
  node["qw"] = transform.transform.rotation.w;
  widget->configure(node);
}

void ExtrinsicHandEyeCalibrationPanel::callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                                                   rclcpp::Client<std_srvs::srv::Trigger>::CallbackType cb)
{
  if (!client->service_is_ready())
  {
    QString message;
    QTextStream stream(&message);
    stream << "Service '" << QString::fromStdString(client->get_service_name()) << "' is not available.";
    QMessageBox::warning(this, "Communication Error", message);
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(request, cb);
}

void ExtrinsicHandEyeCalibrationPanel::triggerObservationCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();
  if (response->success)
    observation_counter_->display(observation_counter_->value() + 1);
  else
    QMessageBox::warning(this, "Error", QString::fromStdString(response->message));
}

void ExtrinsicHandEyeCalibrationPanel::saveCallback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();
  if (response->success)
  {
    QString message;
    QTextStream stream(&message);
    stream << "Saved calibration data to: " << QString::fromStdString(response->message);
    QMessageBox::information(this, "Success", message);
  }
  else
    QMessageBox::warning(this, "Error", QString::fromStdString(response->message));
}

void ExtrinsicHandEyeCalibrationPanel::onCalibrationComplete(
    const industrial_calibration::ExtrinsicHandEyeResult& result)
{
  // Publish the calibrated camera frame
  {
    std::string camera_mount_frame;
    std::string camera_frame;
    if (get_parameter<std::string>(CAMERA_MOUNT_FRAME_PARAM, camera_mount_frame) &&
        get_parameter<std::string>(CAMERA_FRAME_PARAM, camera_frame))
    {
      geometry_msgs::msg::TransformStamped msg = tf2::eigenToTransform(result.camera_mount_to_camera);
      msg.header.frame_id = camera_mount_frame;
      msg.child_frame_id = camera_frame + "_calibrated";
      broadcaster_->sendTransform(msg);
    }
  }

  // Publish the calibrated target frame
  {
    std::string target_mount_frame;
    std::string target_frame;
    if (get_parameter<std::string>(TARGET_MOUNT_FRAME_PARAM, target_mount_frame) &&
        get_parameter<std::string>(TARGET_FRAME_PARAM, target_frame))
    {
      geometry_msgs::msg::TransformStamped msg = tf2::eigenToTransform(result.target_mount_to_target);
      msg.header.frame_id = target_mount_frame;
      msg.child_frame_id = target_frame + "_calibrated";
      broadcaster_->sendTransform(msg);
    }
  }
}

}  // namespace industrial_calibration_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(industrial_calibration_ros::ExtrinsicHandEyeCalibrationPanel, rviz_common::Panel)
