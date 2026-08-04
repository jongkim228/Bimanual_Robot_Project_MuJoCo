#include <panda_bottle_pose_controller/bottle_pose_controller.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

namespace panda_bottle_pose_controller {

namespace {

Eigen::Matrix4d mat(const std::array<double, 16>& a) {
  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> m(a.data());
  return Eigen::Matrix4d(m);
}

std::array<double, 16> arr(const Eigen::Matrix4d& m) {
  std::array<double, 16> a{};
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(a.data()) = m;
  return a;
}

Eigen::Quaterniond quat(const TargetPose& t) {
  Eigen::Quaterniond q(t.q[0], t.q[1], t.q[2], t.q[3]);
  return q.normalized();
}

double angle(Eigen::Quaterniond a, Eigen::Quaterniond b) {
  double d = std::abs(a.normalized().dot(b.normalized()));
  d = std::max(0.0, std::min(1.0, d));
  return 2.0 * std::acos(d);
}

}  // namespace

bool BottlePoseController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh) {
  if (!nh.getParam("arm_id", arm_id_)) {
    ROS_ERROR("Missing arm_id");
    return false;
  }
  nh.param<std::string>("base_frame", base_frame_, arm_id_ + "_link0");
  nh.param("motion_duration", duration_, 10.0);
  nh.param("max_translation", max_translation_, 0.20);
  nh.param("max_rotation", max_rotation_, 0.80);

  std::vector<double> p, q;
  if (!nh.getParam("target_xyz", p) || p.size() != 3 ||
      !nh.getParam("target_quaternion", q) || q.size() != 4) {
    ROS_ERROR("target_xyz must have 3 values and target_quaternion 4 values [w,x,y,z]");
    return false;
  }

  TargetPose t;
  std::copy(p.begin(), p.end(), t.p.begin());
  std::copy(q.begin(), q.end(), t.q.begin());

  std::string why;
  if (!valid(t, &why)) {
    ROS_ERROR_STREAM(why);
    return false;
  }
  target_.writeFromNonRT(t);

  auto* pi = hw->get<franka_hw::FrankaPoseCartesianInterface>();
  auto* si = hw->get<franka_hw::FrankaStateInterface>();
  if (!pi || !si) {
    ROS_ERROR("Required Franka interfaces unavailable");
    return false;
  }

  try {
    pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        pi->getHandle(arm_id_ + "_robot"));
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        si->getHandle(arm_id_ + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  sub_ = nh.subscribe("target_pose", 1, &BottlePoseController::targetCb, this);
  start_srv_ = nh.advertiseService("start", &BottlePoseController::start, this);
  stop_srv_ = nh.advertiseService("stop", &BottlePoseController::stop, this);

  ROS_INFO("Controller initialized; it will hold until start service is called");
  return true;
}

void BottlePoseController::starting(const ros::Time&) {
  auto s = state_handle_->getRobotState();
  hold_pose_ = s.O_T_EE_d;

  std::ostringstream os;
  for (int i = 0; i < 16; ++i) os << s.O_T_EE_d[i] << " ";
  ROS_WARN_STREAM("O_T_EE_d = " << os.str());

  start_pose_ = hold_pose_;
  active_ = false;
  start_req_ = false;
  stop_req_ = false;
  pose_handle_->setCommand(hold_pose_);
}

void BottlePoseController::update(const ros::Time& now, const ros::Duration&) {
  if (stop_req_.exchange(false)) {
    hold_pose_ = state_handle_->getRobotState().O_T_EE_d;
    active_ = false;
    pose_handle_->setCommand(hold_pose_);
    return;
  }

  if (start_req_.exchange(false)) {
    start_pose_ = state_handle_->getRobotState().O_T_EE_d;
    hold_pose_ = start_pose_;
    active_target_ = *target_.readFromRT();

    auto M = mat(start_pose_);
    Eigen::Vector3d p0 = M.block<3, 1>(0, 3);
    Eigen::Vector3d p1(active_target_.p[0], active_target_.p[1], active_target_.p[2]);
    Eigen::Quaterniond q0(M.block<3, 3>(0, 0));
    Eigen::Quaterniond q1 = quat(active_target_);

    double d = (p1 - p0).norm();
    double r = angle(q0, q1);

    if (d > max_translation_ || r > max_rotation_) {
      ROS_ERROR_STREAM("Rejected: translation=" << d << " m, rotation=" << r << " rad");
      active_ = false;
    } else {
      t0_ = now;
      active_ = true;
      ROS_INFO_STREAM("Accepted target: translation=" << d << " m, rotation=" << r << " rad");
    }
  }

  if (!active_) {
    pose_handle_->setCommand(hold_pose_);
    return;
  }

  double s = std::max(0.0, std::min(1.0, (now - t0_).toSec() / duration_));
  double u = blend(s);

  auto M = mat(start_pose_);
  Eigen::Vector3d p0 = M.block<3, 1>(0, 3);
  Eigen::Vector3d p1(active_target_.p[0], active_target_.p[1], active_target_.p[2]);
  Eigen::Quaterniond q0(M.block<3, 3>(0, 0));
  q0.normalize();
  Eigen::Quaterniond q1 = quat(active_target_);
  if (q0.dot(q1) < 0) q1.coeffs() *= -1;  // 최단 경로 보정

  Eigen::Matrix4d C = Eigen::Matrix4d::Identity();
  C.block<3, 3>(0, 0) = q0.slerp(u, q1).normalized().toRotationMatrix();
  C.block<3, 1>(0, 3) = p0 + u * (p1 - p0);

  hold_pose_ = arr(C);
  pose_handle_->setCommand(hold_pose_);

  if (s >= 1) {
    active_ = false;
    ROS_INFO("Target reached");
  }
}

void BottlePoseController::stopping(const ros::Time&) {
  active_ = false;
}

bool BottlePoseController::start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
  if (active_) {
    r.success = false;
    r.message = "motion active";
  } else {
    start_req_ = true;
    r.success = true;
    r.message = "start requested";
  }
  return true;
}

bool BottlePoseController::stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
  stop_req_ = true;
  r.success = true;
  r.message = "stop requested";
  return true;
}

void BottlePoseController::targetCb(const geometry_msgs::PoseStampedConstPtr& m) {
  if (!m->header.frame_id.empty() && m->header.frame_id != base_frame_) {
    ROS_ERROR_STREAM("Expected frame " << base_frame_);
    return;
  }

  TargetPose t;
  t.p = {{m->pose.position.x, m->pose.position.y, m->pose.position.z}};
  t.q = {{m->pose.orientation.w, m->pose.orientation.x,
          m->pose.orientation.y, m->pose.orientation.z}};

  std::string why;
  if (!valid(t, &why)) {
    ROS_ERROR_STREAM(why);
    return;
  }

  target_.writeFromNonRT(t);
  ROS_INFO("Stored new target");
}

bool BottlePoseController::valid(const TargetPose& t, std::string* why) const {
  for (double v : t.p) {
    if (!std::isfinite(v)) {
      *why = "non-finite position";
      return false;
    }
  }

  double n = 0;
  for (double v : t.q) {
    if (!std::isfinite(v)) {
      *why = "non-finite quaternion";
      return false;
    }
    n += v * v;
  }
  if (n < 1e-12) {
    *why = "zero quaternion";
    return false;
  }

  double r = std::sqrt(t.p[0] * t.p[0] + t.p[1] * t.p[1] + t.p[2] * t.p[2]);
  if (r < 0.15 || r > 0.85) {
    std::ostringstream s;
    s << "target radius " << r << " outside broad sanity range [0.15,0.85] m";
    *why = s.str();
    return false;
  }

  return true;
}

double BottlePoseController::blend(double s) {
  return s * s * s * (10 + s * (-15 + 6 * s));
}

}

PLUGINLIB_EXPORT_CLASS(panda_bottle_pose_controller::BottlePoseController,
                        controller_interface::ControllerBase)