#include <left_panda_fixed_pose_test/fixed_pose_controller.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace left_panda_fixed_pose_test {

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Quaterniond normalisedQuaternion(
    const Eigen::Matrix3d& rotation_matrix) {
  Eigen::Quaterniond quaternion(rotation_matrix);
  quaternion.normalize();
  return quaternion;
}

double quaternionAngularDistance(const Eigen::Quaterniond& q1,
                                 const Eigen::Quaterniond& q2) {
  double dot = std::abs(q1.dot(q2));
  dot = std::clamp(dot, 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

}  // namespace

bool FixedPoseController::init(hardware_interface::RobotHW* robot_hw,
                               ros::NodeHandle& node_handle) {
  std::string arm_id;

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("FixedPoseController: missing parameter 'arm_id'.");
    return false;
  }

  base_frame_ = arm_id + "_link0";
  node_handle.param<std::string>("base_frame", base_frame_, base_frame_);

  node_handle.param("motion_duration",
                    motion_duration_,
                    motion_duration_);

  node_handle.param("maximum_translation",
                    maximum_translation_,
                    maximum_translation_);

  node_handle.param("maximum_rotation",
                    maximum_rotation_,
                    maximum_rotation_);

  node_handle.param("maximum_linear_velocity",
                    maximum_linear_velocity_,
                    maximum_linear_velocity_);

  node_handle.param("maximum_angular_velocity",
                    maximum_angular_velocity_,
                    maximum_angular_velocity_);

  node_handle.param("workspace/min_x",
                    workspace_min_x_,
                    workspace_min_x_);

  node_handle.param("workspace/max_x",
                    workspace_max_x_,
                    workspace_max_x_);

  node_handle.param("workspace/min_y",
                    workspace_min_y_,
                    workspace_min_y_);

  node_handle.param("workspace/max_y",
                    workspace_max_y_,
                    workspace_max_y_);

  node_handle.param("workspace/min_z",
                    workspace_min_z_,
                    workspace_min_z_);

  node_handle.param("workspace/max_z",
                    workspace_max_z_,
                    workspace_max_z_);

  if (motion_duration_ < 1.0) {
    ROS_ERROR("FixedPoseController: motion_duration must be >= 1 second.");
    return false;
  }

  auto* pose_interface =
      robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();

  if (pose_interface == nullptr) {
    ROS_ERROR(
        "FixedPoseController: could not obtain "
        "FrankaPoseCartesianInterface.");
    return false;
  }

  try {
    cartesian_pose_handle_ =
        std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
            pose_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& exception) {
    ROS_ERROR_STREAM(
        "FixedPoseController: failed to get Cartesian pose handle: "
        << exception.what());
    return false;
  }

  auto* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();

  if (state_interface == nullptr) {
    ROS_ERROR(
        "FixedPoseController: could not obtain FrankaStateInterface.");
    return false;
  }

  try {
    state_handle_ =
        std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& exception) {
    ROS_ERROR_STREAM(
        "FixedPoseController: failed to get state handle: "
        << exception.what());
    return false;
  }

  std::vector<double> target_position;
  std::vector<double> target_orientation;

  if (node_handle.getParam("target/position", target_position) &&
      node_handle.getParam("target/orientation_xyzw",
                           target_orientation)) {
    if (target_position.size() != 3 ||
        target_orientation.size() != 4) {
      ROS_ERROR(
          "FixedPoseController: target position must have 3 values "
          "and orientation_xyzw must have 4 values.");
      return false;
    }

    Eigen::Quaterniond quaternion(
        target_orientation[3],
        target_orientation[0],
        target_orientation[1],
        target_orientation[2]);

    if (quaternion.norm() < 1.0e-6) {
      ROS_ERROR("FixedPoseController: invalid zero target quaternion.");
      return false;
    }

    quaternion.normalize();

    Eigen::Affine3d target = Eigen::Affine3d::Identity();
    target.translation() =
        Eigen::Vector3d(target_position[0],
                        target_position[1],
                        target_position[2]);
    target.linear() = quaternion.toRotationMatrix();

    configured_target_pose_ = affineToArray(target);
    target_available_ = true;
  }

  start_service_ =
      node_handle.advertiseService(
          "start",
          &FixedPoseController::startCallback,
          this);

  stop_service_ =
      node_handle.advertiseService(
          "stop",
          &FixedPoseController::stopCallback,
          this);

  return_home_service_ =
      node_handle.advertiseService(
          "return_home",
          &FixedPoseController::returnHomeCallback,
          this);

  target_pose_subscriber_ =
      node_handle.subscribe(
          "target_pose",
          1,
          &FixedPoseController::targetPoseCallback,
          this);

  ROS_INFO_STREAM(
      "FixedPoseController initialised for arm_id='"
      << arm_id << "', base_frame='" << base_frame_
      << "', duration=" << motion_duration_ << " s.");

  return true;
}

void FixedPoseController::starting(const ros::Time& time) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  const franka::RobotState robot_state =
      state_handle_->getRobotState();

  /*
   * Use O_T_EE_d rather than a zero or configured pose when the
   * controller starts. This avoids an immediate command discontinuity.
   */
  home_pose_ = robot_state.O_T_EE_d;
  hold_pose_ = home_pose_;
  motion_start_pose_ = home_pose_;
  motion_target_pose_ = home_pose_;

  motion_state_ = MotionState::HOLDING;
  motion_start_time_ = time;

  cartesian_pose_handle_->setCommand(hold_pose_);

  ROS_INFO(
      "FixedPoseController started. Current desired pose saved as home.");
}

void FixedPoseController::update(const ros::Time& time,
                                 const ros::Duration& /*period*/) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  if (motion_state_ == MotionState::HOLDING) {
    cartesian_pose_handle_->setCommand(hold_pose_);
    return;
  }

  const double elapsed =
      std::max(0.0, (time - motion_start_time_).toSec());

  const double u =
      std::clamp(elapsed / motion_duration_, 0.0, 1.0);

  const double s = minimumJerk(u);

  const Eigen::Affine3d start =
      arrayToAffine(motion_start_pose_);

  const Eigen::Affine3d target =
      arrayToAffine(motion_target_pose_);

  Eigen::Affine3d command = Eigen::Affine3d::Identity();

  command.translation() =
      start.translation() +
      s * (target.translation() - start.translation());

  Eigen::Quaterniond q_start =
      normalisedQuaternion(start.linear());

  Eigen::Quaterniond q_target =
      normalisedQuaternion(target.linear());

  if (q_start.dot(q_target) < 0.0) {
    q_target.coeffs() *= -1.0;
  }

  const Eigen::Quaterniond q_command =
      q_start.slerp(s, q_target).normalized();

  command.linear() = q_command.toRotationMatrix();

  const std::array<double, 16> command_array =
      affineToArray(command);

  cartesian_pose_handle_->setCommand(command_array);

  if (u >= 1.0) {
    hold_pose_ = motion_target_pose_;
    motion_state_ = MotionState::HOLDING;

    ROS_INFO(
        "FixedPoseController: motion completed; holding final pose.");
  }
}

void FixedPoseController::stopping(const ros::Time& /*time*/) {
  ROS_INFO("FixedPoseController stopped.");
}

bool FixedPoseController::startCallback(
    std_srvs::Trigger::Request& /*request*/,
    std_srvs::Trigger::Response& response) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  if (!target_available_) {
    response.success = false;
    response.message =
        "No valid target pose is configured or received.";
    return true;
  }

  const franka::RobotState state =
      state_handle_->getRobotState();

  std::string reason;

  if (!initialiseMotion(
          state.O_T_EE_d,
          configured_target_pose_,
          "target",
          reason)) {
    response.success = false;
    response.message = reason;
    return true;
  }

  motion_state_ = MotionState::MOVING_TO_TARGET;
  motion_start_time_ = ros::Time::now();

  response.success = true;
  response.message =
      "Motion to configured target started.";

  return true;
}

bool FixedPoseController::stopCallback(
    std_srvs::Trigger::Request& /*request*/,
    std_srvs::Trigger::Response& response) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  const franka::RobotState state =
      state_handle_->getRobotState();

  /*
   * Hold the current desired pose, not a distant old target.
   * This is a controlled hold request, not an emergency stop.
   */
  hold_pose_ = state.O_T_EE_d;
  motion_state_ = MotionState::HOLDING;

  response.success = true;
  response.message =
      "Trajectory stopped; holding current desired pose.";

  return true;
}

bool FixedPoseController::returnHomeCallback(
    std_srvs::Trigger::Request& /*request*/,
    std_srvs::Trigger::Response& response) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  const franka::RobotState state =
      state_handle_->getRobotState();

  std::string reason;

  if (!initialiseMotion(
          state.O_T_EE_d,
          home_pose_,
          "home",
          reason)) {
    response.success = false;
    response.message = reason;
    return true;
  }

  motion_state_ = MotionState::RETURNING_HOME;
  motion_start_time_ = ros::Time::now();

  response.success = true;
  response.message = "Return-home motion started.";

  return true;
}

void FixedPoseController::targetPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (!message->header.frame_id.empty() &&
      message->header.frame_id != base_frame_) {
    ROS_ERROR_STREAM(
        "FixedPoseController: target frame is '"
        << message->header.frame_id
        << "', but expected '" << base_frame_ << "'.");
    return;
  }

  const auto& p = message->pose.position;
  const auto& o = message->pose.orientation;

  Eigen::Quaterniond quaternion(o.w, o.x, o.y, o.z);

  if (quaternion.norm() < 1.0e-6) {
    ROS_ERROR(
        "FixedPoseController: rejected target with zero quaternion.");
    return;
  }

  quaternion.normalize();

  Eigen::Affine3d target = Eigen::Affine3d::Identity();
  target.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  target.linear() = quaternion.toRotationMatrix();

  const Eigen::Vector3d position = target.translation();

  if (position.x() < workspace_min_x_ ||
      position.x() > workspace_max_x_ ||
      position.y() < workspace_min_y_ ||
      position.y() > workspace_max_y_ ||
      position.z() < workspace_min_z_ ||
      position.z() > workspace_max_z_) {
    ROS_ERROR_STREAM(
        "FixedPoseController: rejected target outside workspace: ["
        << position.transpose() << "].");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    configured_target_pose_ = affineToArray(target);
    target_available_ = true;
  }

  ROS_INFO_STREAM(
      "FixedPoseController: accepted target pose ["
      << position.transpose() << "] in frame "
      << base_frame_ << ".");
}

bool FixedPoseController::initialiseMotion(
    const std::array<double, 16>& start_pose,
    const std::array<double, 16>& target_pose,
    const std::string& motion_name,
    std::string& reason) {
  const Eigen::Affine3d start = arrayToAffine(start_pose);
  const Eigen::Affine3d target = arrayToAffine(target_pose);

  const Eigen::Vector3d target_position =
      target.translation();

  if (target_position.x() < workspace_min_x_ ||
      target_position.x() > workspace_max_x_ ||
      target_position.y() < workspace_min_y_ ||
      target_position.y() > workspace_max_y_ ||
      target_position.z() < workspace_min_z_ ||
      target_position.z() > workspace_max_z_) {
    reason =
        "Target is outside the configured Cartesian workspace.";
    return false;
  }

  const double translation =
      (target.translation() - start.translation()).norm();

  const Eigen::Quaterniond q_start =
      normalisedQuaternion(start.linear());

  const Eigen::Quaterniond q_target =
      normalisedQuaternion(target.linear());

  const double rotation =
      quaternionAngularDistance(q_start, q_target);

  if (translation > maximum_translation_) {
    reason =
        "Translation " + std::to_string(translation) +
        " m exceeds maximum_translation " +
        std::to_string(maximum_translation_) + " m.";
    return false;
  }

  if (rotation > maximum_rotation_) {
    reason =
        "Rotation " + std::to_string(rotation) +
        " rad exceeds maximum_rotation " +
        std::to_string(maximum_rotation_) + " rad.";
    return false;
  }

  /*
   * For minimum jerk, max ds/dt = 1.875 / duration.
   */
  const double estimated_max_linear_velocity =
      1.875 * translation / motion_duration_;

  const double estimated_max_angular_velocity =
      1.875 * rotation / motion_duration_;

  if (estimated_max_linear_velocity >
      maximum_linear_velocity_) {
    reason =
        "Estimated peak linear velocity " +
        std::to_string(estimated_max_linear_velocity) +
        " m/s exceeds configured maximum.";
    return false;
  }

  if (estimated_max_angular_velocity >
      maximum_angular_velocity_) {
    reason =
        "Estimated peak angular velocity " +
        std::to_string(estimated_max_angular_velocity) +
        " rad/s exceeds configured maximum.";
    return false;
  }

  motion_start_pose_ = start_pose;
  motion_target_pose_ = target_pose;

  ROS_INFO_STREAM(
      "FixedPoseController: prepared " << motion_name
      << " motion. Translation=" << translation
      << " m, rotation=" << rotation
      << " rad, duration=" << motion_duration_ << " s.");

  return true;
}

Eigen::Affine3d FixedPoseController::arrayToAffine(
    const std::array<double, 16>& pose) {
  /*
   * libfranka transforms are column-major, as are Eigen matrices.
   */
  return Eigen::Affine3d(
      Eigen::Matrix4d::Map(pose.data()));
}

std::array<double, 16> FixedPoseController::affineToArray(
    const Eigen::Affine3d& pose) {
  std::array<double, 16> result{};

  Eigen::Map<Eigen::Matrix4d>(result.data()) =
      pose.matrix();

  return result;
}

double FixedPoseController::minimumJerk(double u) {
  u = std::clamp(u, 0.0, 1.0);

  return 10.0 * std::pow(u, 3) -
         15.0 * std::pow(u, 4) +
          6.0 * std::pow(u, 5);
}

double FixedPoseController::minimumJerkDerivative(double u) {
  u = std::clamp(u, 0.0, 1.0);

  return 30.0 * std::pow(u, 2) -
         60.0 * std::pow(u, 3) +
         30.0 * std::pow(u, 4);
}

}  // namespace left_panda_fixed_pose_test

PLUGINLIB_EXPORT_CLASS(
    left_panda_fixed_pose_test::FixedPoseController,
    controller_interface::ControllerBase)
