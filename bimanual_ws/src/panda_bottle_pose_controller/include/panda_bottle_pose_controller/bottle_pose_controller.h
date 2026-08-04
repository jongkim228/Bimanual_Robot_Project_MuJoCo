#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <std_srvs/Trigger.h>
namespace panda_bottle_pose_controller {
struct TargetPose { std::array<double,3> p{{0,0,0}}; std::array<double,4> q{{1,0,0,0}}; };
class BottlePoseController : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW*, ros::NodeHandle&) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration&) override;
  void stopping(const ros::Time&) override;
 private:
  bool start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  bool stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  void targetCb(const geometry_msgs::PoseStampedConstPtr&);
  bool valid(const TargetPose&, std::string*) const;
  static double blend(double);
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> pose_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  realtime_tools::RealtimeBuffer<TargetPose> target_;
  ros::Subscriber sub_; ros::ServiceServer start_srv_, stop_srv_;
  std::string arm_id_, base_frame_;
  double duration_{10.0}, max_translation_{0.20}, max_rotation_{0.80};
  std::array<double,16> start_pose_{}, hold_pose_{};
  ros::Time t0_; std::atomic<bool> start_req_{false}, stop_req_{false}; bool active_{false}; TargetPose active_target_;
};
}