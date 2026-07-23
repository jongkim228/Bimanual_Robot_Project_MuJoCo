#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_srvs/Trigger.h>

namespace left_panda_fixed_pose_test {

class FixedPoseController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaPoseCartesianInterface,
          franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle) override;

  void starting(const ros::Time& time) override;

  void update(const ros::Time& time,
              const ros::Duration& period) override;

  void stopping(const ros::Time& time) override;

 private:
  enum class MotionState {
    HOLDING,
    MOVING_TO_TARGET,
    RETURNING_HOME
  };

  bool startCallback(std_srvs::Trigger::Request& request,
                     std_srvs::Trigger::Response& response);

  bool stopCallback(std_srvs::Trigger::Request& request,
                    std_srvs::Trigger::Response& response);

  bool returnHomeCallback(std_srvs::Trigger::Request& request,
                          std_srvs::Trigger::Response& response);

  void targetPoseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& message);

  bool initialiseMotion(const std::array<double, 16>& start_pose,
                        const std::array<double, 16>& target_pose,
                        const std::string& motion_name,
                        std::string& reason);

  static Eigen::Affine3d arrayToAffine(
      const std::array<double, 16>& pose);

  static std::array<double, 16> affineToArray(
      const Eigen::Affine3d& pose);

  static double minimumJerk(double u);

  static double minimumJerkDerivative(double u);

  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle>
      cartesian_pose_handle_;

  std::unique_ptr<franka_hw::FrankaStateHandle>
      state_handle_;

  ros::ServiceServer start_service_;
  ros::ServiceServer stop_service_;
  ros::ServiceServer return_home_service_;
  ros::Subscriber target_pose_subscriber_;

  std::mutex command_mutex_;

  std::array<double, 16> home_pose_{};
  std::array<double, 16> hold_pose_{};
  std::array<double, 16> motion_start_pose_{};
  std::array<double, 16> motion_target_pose_{};
  std::array<double, 16> configured_target_pose_{};

  MotionState motion_state_{MotionState::HOLDING};

  ros::Time motion_start_time_;

  double motion_duration_{10.0};
  double maximum_translation_{0.20};
  double maximum_rotation_{0.80};
  double maximum_linear_velocity_{0.05};
  double maximum_angular_velocity_{0.30};

  double workspace_min_x_{0.15};
  double workspace_max_x_{0.85};
  double workspace_min_y_{-0.60};
  double workspace_max_y_{0.60};
  double workspace_min_z_{0.05};
  double workspace_max_z_{0.85};

  std::string base_frame_;
  bool target_available_{false};
};

}  // namespace left_panda_fixed_pose_test
