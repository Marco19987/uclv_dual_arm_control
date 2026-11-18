// Trajectory generator: joint-space and Cartesian quintic trajectories exposed as ROS 2 actions.
//
//  action  generate_joint_trajectory      (dual_arm_control_interfaces/action/JointTrajectory)
//            -> publishes command/joint_states       (sensor_msgs/JointState)
//  action  generate_cartesian_trajectory  (dual_arm_control_interfaces/action/CartesianTrajectory)
//            -> publishes cartesian_traj/pose        (geometry_msgs/PoseStamped)
//                         cartesian_traj/twist       (geometry_msgs/TwistStamped, base-frame twist)
//
// Each segment between two waypoints is a quintic polynomial. For Cartesian trajectories the position follows a
// quintic per axis; the orientation rotates about the fixed axis of Qi^-1 Qf with a rest-to-rest quintic angle.
// A new goal preempts the running one. The goal header stamp is the start time (0 = now); before it the initial
// sample is published.

#include <atomic>
#include <csignal>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "dual_arm_control/quintic.hpp"
#include "dual_arm_control_interfaces/action/cartesian_trajectory.hpp"
#include "dual_arm_control_interfaces/action/joint_trajectory.hpp"

using dual_arm_control::QuinticPoly;
using JointTraj = dual_arm_control_interfaces::action::JointTrajectory;
using CartTraj = dual_arm_control_interfaces::action::CartesianTrajectory;

namespace
{
double toSec(const builtin_interfaces::msg::Duration& d)
{
  return rclcpp::Duration(d).seconds();
}

/// Keeps one active goal per action: a new goal aborts the previous one.
template <typename ActionT>
struct ActiveGoal
{
  std::mutex mutex;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle;
  std::atomic<uint64_t> generation{ 0 };
};
}  // namespace

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  explicit TrajectoryGeneratorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("trajectory_generator", options)
  {
    rate_ = declare_parameter<double>("trajectory.rate", 200.0);
    const bool enable_joint = declare_parameter<bool>("enable_joint_trajectory", true);
    const bool enable_cartesian = declare_parameter<bool>("enable_cartesian_trajectory", true);

    if (enable_joint)
    {
      joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("command/joint_states", 1);
      joint_server_ = rclcpp_action::create_server<JointTraj>(
          this, "generate_joint_trajectory",
          [](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
          [](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
          [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<JointTraj>> gh) {
            start(joint_goal_, gh, [this](auto h, uint64_t gen) { executeJoint(h, gen); });
          });
    }
    if (enable_cartesian)
    {
      pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("cartesian_traj/pose", 1);
      twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cartesian_traj/twist", 1);
      cart_server_ = rclcpp_action::create_server<CartTraj>(
          this, "generate_cartesian_trajectory",
          [](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
          [](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
          [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CartTraj>> gh) {
            start(cart_goal_, gh, [this](auto h, uint64_t gen) { executeCartesian(h, gen); });
          });
    }
  }

private:
  template <typename ActionT, typename Fn>
  void start(ActiveGoal<ActionT>& active, std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh, Fn fn)
  {
    uint64_t gen;
    {
      std::lock_guard<std::mutex> lock(active.mutex);
      if (active.handle && active.handle->is_active())
      {
        auto result = std::make_shared<typename ActionT::Result>();
        result->message = "Preempted by a new goal";
        active.handle->abort(result);
      }
      active.handle = gh;
      gen = ++active.generation;
    }
    std::lock_guard<std::mutex> lock(threads_mutex_);
    threads_.emplace_back([fn, gh, gen]() {
      try
      {
        fn(gh, gen);
      }
      catch (const std::exception& e)
      {
        // ROS was shut down while the trajectory was running
        if (rclcpp::ok())
          RCLCPP_ERROR(rclcpp::get_logger("trajectory_generator"), "Trajectory execution failed: %s", e.what());
      }
      // A goal handle destroyed while still active tries to abort itself and throws if the action server is gone
      // (shutdown): always leave the goal in a terminal state here.
      try
      {
        if (gh->is_active())
        {
          auto result = std::make_shared<typename ActionT::Result>();
          result->message = "Trajectory interrupted";
          gh->abort(result);
        }
      }
      catch (const std::exception&)
      {
      }
    });
  }

  template <typename ActionT>
  bool stillOwner(ActiveGoal<ActionT>& active, uint64_t gen)
  {
    return active.generation == gen && rclcpp::ok();
  }

  rclcpp::Time startTime(const builtin_interfaces::msg::Time& stamp)
  {
    rclcpp::Time t0(stamp, get_clock()->get_clock_type());
    return (t0.nanoseconds() == 0) ? now() : t0;
  }

  // ---------------------------------------------------------------------------------------------------------------
  void executeJoint(std::shared_ptr<rclcpp_action::ServerGoalHandle<JointTraj>> gh, uint64_t gen)
  {
    const auto goal = gh->get_goal();
    const auto& pts = goal->trajectory.points;
    auto result = std::make_shared<JointTraj::Result>();
    if (pts.empty())
    {
      result->message = "Empty trajectory";
      gh->succeed(result);
      return;
    }
    if (pts.size() < 2)
    {
      result->message = "At least two points are required";
      gh->abort(result);
      return;
    }
    const size_t n = pts[0].positions.size();
    auto at = [](const std::vector<double>& v, size_t j) { return j < v.size() ? v[j] : 0.0; };

    const double t0 = startTime(goal->trajectory.header.stamp).seconds();
    const double tf_total = t0 + toSec(pts.back().time_from_start);
    rclcpp::WallRate rate(rate_);
    auto feedback = std::make_shared<JointTraj::Feedback>();

    sensor_msgs::msg::JointState msg;
    msg.name = goal->trajectory.joint_names;
    msg.position.resize(n);
    msg.velocity.resize(n);
    msg.effort.assign(n, 0.0);

    for (size_t i = 1; i < pts.size(); ++i)
    {
      const double ts = t0 + toSec(pts[i - 1].time_from_start);
      const double T = toSec(pts[i].time_from_start) - toSec(pts[i - 1].time_from_start);
      std::vector<QuinticPoly> q(n);
      for (size_t j = 0; j < n; ++j)
        q[j] = QuinticPoly(T, pts[i - 1].positions[j], pts[i].positions[j], ts, at(pts[i - 1].velocities, j),
                           at(pts[i].velocities, j), at(pts[i - 1].accelerations, j), at(pts[i].accelerations, j));
      while (true)
      {
        if (!stillOwner(joint_goal_, gen))
          return;
        if (gh->is_canceling())
        {
          result->message = "Preemption requested";
          gh->canceled(result);
          return;
        }
        const double t = now().seconds();
        if (t >= ts + T)
          break;
        for (size_t j = 0; j < n; ++j)
        {
          msg.position[j] = q[j].position(t);
          msg.velocity[j] = q[j].velocity(t);
        }
        msg.header.stamp = now();
        joint_pub_->publish(msg);
        feedback->time_left = rclcpp::Duration::from_seconds(tf_total - t);
        gh->publish_feedback(feedback);
        rate.sleep();
      }
    }
    msg.position = pts.back().positions;
    msg.velocity.assign(n, 0.0);
    for (size_t j = 0; j < n; ++j)
      msg.velocity[j] = at(pts.back().velocities, j);
    msg.header.stamp = now();
    joint_pub_->publish(msg);
    if (stillOwner(joint_goal_, gen))
    {
      result->message = "Trajectory completed";
      gh->succeed(result);
    }
  }

  // ---------------------------------------------------------------------------------------------------------------
  void publishCartesian(const Eigen::Vector3d& p, const Eigen::Quaterniond& Q, const Eigen::Vector3d& v,
                        const Eigen::Vector3d& w)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.w = Q.w();
    pose.pose.orientation.x = Q.x();
    pose.pose.orientation.y = Q.y();
    pose.pose.orientation.z = Q.z();
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = pose.header.stamp;
    twist.twist.linear.x = v.x();
    twist.twist.linear.y = v.y();
    twist.twist.linear.z = v.z();
    twist.twist.angular.x = w.x();
    twist.twist.angular.y = w.y();
    twist.twist.angular.z = w.z();
    pose_pub_->publish(pose);
    twist_pub_->publish(twist);
  }

  void executeCartesian(std::shared_ptr<rclcpp_action::ServerGoalHandle<CartTraj>> gh, uint64_t gen)
  {
    const auto goal = gh->get_goal();
    const auto& pts = goal->trajectory.points;
    auto result = std::make_shared<CartTraj::Result>();
    if (pts.empty())
    {
      result->message = "Empty trajectory";
      gh->succeed(result);
      return;
    }
    if (pts.size() < 2)
    {
      result->message = "At least two points are required";
      gh->abort(result);
      return;
    }
    auto pos = [](const auto& pt) { return Eigen::Vector3d(pt.pose.position.x, pt.pose.position.y, pt.pose.position.z); };
    auto quat = [](const auto& pt) {
      return Eigen::Quaterniond(pt.pose.orientation.w, pt.pose.orientation.x, pt.pose.orientation.y,
                                pt.pose.orientation.z);
    };
    auto lin = [](const geometry_msgs::msg::Vector3& v) { return Eigen::Vector3d(v.x, v.y, v.z); };

    const double t0 = startTime(goal->trajectory.header.stamp).seconds();
    const double tf_total = t0 + toSec(pts.back().time_from_start);
    rclcpp::WallRate rate(rate_);
    auto feedback = std::make_shared<CartTraj::Feedback>();
    Eigen::Quaterniond Q_last = quat(pts[0]);

    for (size_t i = 1; i < pts.size(); ++i)
    {
      const double ts = t0 + toSec(pts[i - 1].time_from_start);
      const double T = toSec(pts[i].time_from_start) - toSec(pts[i - 1].time_from_start);
      const Eigen::Vector3d p0 = pos(pts[i - 1]), p1 = pos(pts[i]);
      const Eigen::Vector3d v0 = lin(pts[i - 1].velocity.linear), v1 = lin(pts[i].velocity.linear);
      const Eigen::Vector3d a0 = lin(pts[i - 1].acceleration.linear), a1 = lin(pts[i].acceleration.linear);
      QuinticPoly px[3];
      for (int k = 0; k < 3; ++k)
        px[k] = QuinticPoly(T, p0[k], p1[k], ts, v0[k], v1[k], a0[k], a1[k]);
      const Eigen::Quaterniond Qi = quat(pts[i - 1]), Qf = quat(pts[i]);
      const Eigen::AngleAxisd aa(Qi.inverse() * Qf);
      const QuinticPoly angle(T, 0.0, aa.angle(), ts);
      const Eigen::Matrix3d Ri = Qi.normalized().toRotationMatrix();

      while (true)
      {
        if (!stillOwner(cart_goal_, gen))
          return;
        if (gh->is_canceling())
        {
          result->message = "Preemption requested";
          gh->canceled(result);
          return;
        }
        const double t = now().seconds();
        if (px[0].isCompleted(t))
          break;
        Eigen::Vector3d p, v;
        for (int k = 0; k < 3; ++k)
        {
          p[k] = px[k].position(t);
          v[k] = px[k].velocity(t);
        }
        Eigen::Quaterniond Q = Qi * Eigen::Quaterniond(Eigen::AngleAxisd(angle.position(t), aa.axis()));
        Q.normalize();
        const Eigen::Vector3d w = Ri * (angle.velocity(t) * aa.axis());
        Q_last = Q;
        publishCartesian(p, Q, v, w);
        feedback->time_left = rclcpp::Duration::from_seconds(tf_total - t);
        gh->publish_feedback(feedback);
        rate.sleep();
      }
    }

    // final sample: last waypoint (quaternion sign aligned with the last published one)
    Eigen::Quaterniond Qf = quat(pts.back());
    if (Qf.vec().dot(Q_last.vec()) < -0.01)
      Qf.coeffs() *= -1.0;
    Qf.normalize();
    publishCartesian(pos(pts.back()), Qf, lin(pts.back().velocity.linear), lin(pts.back().velocity.angular));
    if (stillOwner(cart_goal_, gen))
    {
      result->message = "Trajectory completed";
      gh->succeed(result);
    }
  }

  double rate_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp_action::Server<JointTraj>::SharedPtr joint_server_;
  rclcpp_action::Server<CartTraj>::SharedPtr cart_server_;
  ActiveGoal<JointTraj> joint_goal_;
  ActiveGoal<CartTraj> cart_goal_;
  std::mutex threads_mutex_;
  std::vector<std::thread> threads_;

public:
  /// Wait for the running trajectories, which return as soon as ROS is shut down.
  ~TrajectoryGeneratorNode()
  {
    std::lock_guard<std::mutex> lock(threads_mutex_);
    for (auto& t : threads_)
      if (t.joinable())
        t.join();
  }
};

int main(int argc, char** argv)
{
  // rclcpp installs its own SIGINT/SIGTERM handler and restores the previous one at shutdown. launch / Ctrl+C
  // usually deliver a second SIGINT while the node is cleaning up: with SIG_IGN as previous handler the
  // process exits normally instead of being killed by the default signal action.
  std::signal(SIGINT, SIG_IGN);
  std::signal(SIGTERM, SIG_IGN);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryGeneratorNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();  // makes the running trajectories return
  exec.remove_node(node);
  node.reset();        // joins the trajectory threads
  return 0;
}
