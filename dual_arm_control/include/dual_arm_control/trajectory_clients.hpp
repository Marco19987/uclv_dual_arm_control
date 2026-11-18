#pragma once

// Helpers used by the task sequencer: wait for a single message and send joint / Cartesian trajectory goals
// to trajectory_generator_node.

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "dual_arm_control_interfaces/action/cartesian_trajectory.hpp"
#include "dual_arm_control_interfaces/action/joint_trajectory.hpp"

namespace dual_arm_control
{

/// Block until one message is received on `topic` (sensor-data QoS). Uses a private executor so it can be called
/// while the node is spun elsewhere.
template <typename MessageT>
std::shared_ptr<const MessageT> waitForMessage(const std::string& topic, const rclcpp::Node::SharedPtr& node)
{
  std::shared_ptr<const MessageT> msg;
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  auto sub = node->create_subscription<MessageT>(
      topic, rclcpp::SensorDataQoS(), [&msg](std::shared_ptr<const MessageT> m) { msg = m; }, options);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_callback_group(group, node->get_node_base_interface());
  while (!msg)
  {
    if (!rclcpp::ok())
      throw std::runtime_error("Interrupted - not rclcpp::ok()");
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return msg;
}

inline void poseToEigen(const geometry_msgs::msg::Pose& pose, Eigen::Matrix4d& T)
{
  T.setIdentity();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  T.block<3, 3>(0, 0) =
      Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
          .toRotationMatrix();
}

inline void eigenToPose(const Eigen::Matrix4d& T, geometry_msgs::msg::Pose& pose)
{
  pose.position.x = T(0, 3);
  pose.position.y = T(1, 3);
  pose.position.z = T(2, 3);
  Eigen::Quaterniond q(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
  q.normalize();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
}

/// Generic action client: sends a goal, waits up to 5 s for acceptance and (optionally) for the result.
template <typename ActionT>
class TrajectoryClient
{
public:
  using Goal = typename ActionT::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  TrajectoryClient(const rclcpp::Node::SharedPtr& node, const std::string& action_name) : node_(node)
  {
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    client_ = rclcpp_action::create_client<ActionT>(node_, action_name, group_);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
  }

  bool waitForServer(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
  {
    return client_->wait_for_action_server(timeout);
  }

  std::shared_future<typename GoalHandle::WrappedResult> send(const Goal& goal, bool wait = true)
  {
    client_->wait_for_action_server();
    auto accepted = client_->async_send_goal(goal);
    if (executor_.spin_until_future_complete(accepted, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("Goal acceptance timeout");
    auto handle = accepted.get();
    if (!handle)
      throw std::runtime_error("Goal rejected");
    auto result = client_->async_get_result(handle);
    if (wait)
    {
      executor_.spin_until_future_complete(result);
      switch (result.get().code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          throw std::runtime_error("Goal aborted");
        case rclcpp_action::ResultCode::CANCELED:
          throw std::runtime_error("Goal canceled");
        default:
          throw std::runtime_error("Unknown result code");
      }
    }
    return result;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
};

/// Joint-space point-to-point motions (quintic, rest-to-rest).
class JointTrajectoryClient : public TrajectoryClient<dual_arm_control_interfaces::action::JointTrajectory>
{
public:
  using TrajectoryClient::TrajectoryClient;

  /// Move from the configuration read on `joint_state_topic` to `qf` in `duration`, starting at `t0` (0 = now).
  auto goTo(const std::string& joint_state_topic, const std::vector<double>& qf, const rclcpp::Duration& duration,
            const rclcpp::Time& t0 = rclcpp::Time(0), bool wait = true)
  {
    auto js = waitForMessage<sensor_msgs::msg::JointState>(joint_state_topic, node_);
    std::vector<double> q0(js->position.begin(), js->position.begin() + qf.size());
    Goal goal;
    goal.trajectory.header.stamp = t0;
    goal.trajectory.joint_names = js->name;
    goal.trajectory.joint_names.resize(qf.size());
    goal.trajectory.points.resize(2);
    const std::vector<double> zeros(qf.size(), 0.0);
    goal.trajectory.points[0].positions = q0;
    goal.trajectory.points[0].time_from_start = rclcpp::Duration(0, 0);
    goal.trajectory.points[1].positions = qf;
    goal.trajectory.points[1].time_from_start = duration;
    for (auto& p : goal.trajectory.points)
    {
      p.velocities = zeros;
      p.accelerations = zeros;
    }
    return send(goal, wait);
  }
};

/// Cartesian motions (quintic position, quintic angle about a fixed axis for the orientation).
class CartesianTrajectoryClient : public TrajectoryClient<dual_arm_control_interfaces::action::CartesianTrajectory>
{
public:
  using TrajectoryClient::TrajectoryClient;
  using Point = dual_arm_control_interfaces::msg::CartesianTrajectoryPoint;

  auto goTo(const geometry_msgs::msg::Pose& initial_pose, const geometry_msgs::msg::Pose& final_pose,
            const rclcpp::Duration& duration, const rclcpp::Time& t0 = rclcpp::Time(0), bool wait = true)
  {
    Goal goal;
    goal.trajectory.header.stamp = t0;
    goal.trajectory.points.resize(2);
    goal.trajectory.points[0].pose = initial_pose;
    goal.trajectory.points[0].time_from_start = rclcpp::Duration(0, 0);
    goal.trajectory.points[1].pose = final_pose;
    goal.trajectory.points[1].time_from_start = duration;
    return send(goal, wait);
  }

  /// Move from the pose read on `pose_topic` to `final_pose`.
  auto goTo(const std::string& pose_topic, const geometry_msgs::msg::Pose& final_pose,
            const rclcpp::Duration& duration, const rclcpp::Time& t0 = rclcpp::Time(0), bool wait = true)
  {
    auto current = waitForMessage<geometry_msgs::msg::PoseStamped>(pose_topic, node_);
    return goTo(current->pose, final_pose, duration, t0, wait);
  }

  /// Execute `goal` starting from the pose read on `pose_topic` (prepended as the first waypoint).
  auto goTo(const std::string& pose_topic, Goal goal, bool wait = true)
  {
    auto current = waitForMessage<geometry_msgs::msg::PoseStamped>(pose_topic, node_);
    Point first;
    first.pose = current->pose;
    first.time_from_start = rclcpp::Duration(0, 0);
    goal.trajectory.points.insert(goal.trajectory.points.begin(), first);
    return send(goal, wait);
  }

  /// Move by `relative_pose`, expressed in the current end-effector frame read on `pose_topic`.
  auto goTo_EE(const std::string& pose_topic, const geometry_msgs::msg::Pose& relative_pose,
               const rclcpp::Duration& duration, const rclcpp::Time& t0 = rclcpp::Time(0), bool wait = true)
  {
    auto current = waitForMessage<geometry_msgs::msg::PoseStamped>(pose_topic, node_);
    Eigen::Matrix4d bTe, eTnew;
    poseToEigen(current->pose, bTe);
    poseToEigen(relative_pose, eTnew);
    geometry_msgs::msg::Pose target;
    eigenToPose(bTe * eTnew, target);
    return goTo(current->pose, target, duration, t0, wait);
  }
};

}  // namespace dual_arm_control
