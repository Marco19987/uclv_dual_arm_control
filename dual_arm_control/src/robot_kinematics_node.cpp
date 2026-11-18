// Kinematic simulation of one robot arm (Pinocchio).
//
// The node owns the joint configuration q of the robot and closes the kinematic control loop:
//
//  * joint integrator  (service joint_integrator/set_running, std_srvs/SetBool)
//      running:  q is integrated (trapezoidal rule) from the joint velocity computed by the inverse kinematics
//      stopped:  q follows the joint commands received on command/joint_states (joint trajectories)
//  * inverse differential kinematics in the end-effector (body) frame
//      input  twist_body (geometry_msgs/TwistStamped, [linear; angular] in the end-effector frame)
//      qdot = (H J)^+ H v  +  (I - (H J)^+ (H J)) qdot_null
//      H: selection matrix (parameter selection_matrix_vector), J: body Jacobian of the end effector
//      qdot_null: null-space joint controller (service activate_joint_controller)
//  * forward kinematics
//      joint_states                                     sensor_msgs/JointState
//      fkine                                            end-effector pose in the base frame
//      fkine_camera                                     camera pose in the base frame
//      command/fkine_twist_tactile_sensor_body_frame    end-effector twist in the end-effector frame
//  * services set_end_effector / camera/set_end_effector (flange -> end effector / camera transforms)

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "dual_arm_control_interfaces/srv/activate_joint_controller.hpp"
#include "dual_arm_control_interfaces/srv/set_end_effector.hpp"

using SetEndEffector = dual_arm_control_interfaces::srv::SetEndEffector;
using ActivateJointController = dual_arm_control_interfaces::srv::ActivateJointController;

class RobotKinematicsNode : public rclcpp::Node
{
public:
  explicit RobotKinematicsNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("robot_kinematics", options)
  {
    // ---- model -----------------------------------------------------------------------------------------------
    const auto urdf = declare_parameter<std::string>("robot_description", "");
    const auto flange = declare_parameter<std::string>("flange_frame", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    if (urdf.empty() || flange.empty())
      throw std::runtime_error("parameters robot_description and flange_frame are required");
    pinocchio::urdf::buildModelFromXML(urdf, model_);
    data_ = pinocchio::Data(model_);
    if (!model_.existFrame(flange))
      throw std::runtime_error("flange frame '" + flange + "' not found in the robot description");
    flange_id_ = model_.getFrameId(flange);
    n_ = model_.nv;
    for (pinocchio::JointIndex j = 1; j < model_.joints.size(); ++j)
      if (model_.joints[j].nq() == 1)
        joint_names_.push_back(model_.names[j]);

    // ---- parameters ------------------------------------------------------------------------------------------
    sample_time_ = declare_parameter<double>("sample_time", 0.005);
    vel_limits_ = declare_parameter<std::vector<double>>("joint_vel_limits", std::vector<double>(n_, 1.0));
    null_gain_ = declare_parameter<double>("null_space_gain", 0.3);
    null_saturation_ = declare_parameter<double>("null_space_saturation", 0.17);
    auto q0 = declare_parameter<std::vector<double>>("initial_joint_positions", std::vector<double>(n_, 0.0));
    setSelection(declare_parameter<std::vector<bool>>("selection_matrix_vector", std::vector<bool>(6, true)));
    if (static_cast<int>(vel_limits_.size()) != n_ || static_cast<int>(q0.size()) != n_)
      throw std::runtime_error("joint_vel_limits / initial_joint_positions must have one entry per joint");

    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
      rcl_interfaces::msg::SetParametersResult res;
      res.successful = true;
      for (const auto& p : params)
      {
        if (p.get_name() == "selection_matrix_vector")
        {
          const auto v = p.as_bool_array();
          if (v.size() != 6)
          {
            res.successful = false;
            res.reason = "selection_matrix_vector must have 6 elements";
            return res;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          setSelection(v);
          RCLCPP_INFO(get_logger(), "Selection matrix set to [%d %d %d %d %d %d]", (int)v[0], (int)v[1], (int)v[2],
                      (int)v[3], (int)v[4], (int)v[5]);
        }
      }
      return res;
    });

    q_ = Eigen::Map<Eigen::VectorXd>(q0.data(), n_);
    qdot_ = Eigen::VectorXd::Zero(n_);
    qdot_prev_ = Eigen::VectorXd::Zero(n_);
    q_null_des_ = q_;
    nTe_.setIdentity();
    nTc_.setIdentity();

    // ---- ROS interfaces --------------------------------------------------------------------------------------
    const auto qos = rclcpp::SensorDataQoS();
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
    fkine_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("fkine", qos);
    fkine_camera_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("fkine_camera", qos);
    twist_pub_ =
        create_publisher<geometry_msgs::msg::TwistStamped>("command/fkine_twist_tactile_sensor_body_frame", qos);

    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "command/joint_states", 1, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (static_cast<int>(msg->position.size()) < n_)
            return;
          std::lock_guard<std::mutex> lock(mutex_);
          q_command_ = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), n_);
          has_command_ = true;
        });
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "twist_body", qos, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) { twistCallback(*msg); });

    integrator_srv_ = create_service<std_srvs::srv::SetBool>(
        "joint_integrator/set_running", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          std::lock_guard<std::mutex> lock(mutex_);
          res->success = true;
          if (req->data && integrator_running_)
            res->message = "Integrator already running";
          else if (req->data)
          {
            integrator_running_ = true;
            has_qdot_ = false;
            qdot_.setZero();
            qdot_prev_.setZero();
            res->message = "Integrator started";
          }
          else
          {
            integrator_running_ = false;
            has_qdot_ = false;
            has_command_ = false;
            res->message = "Integrator stopped";
          }
        });
    ee_srv_ = create_service<SetEndEffector>(
        "set_end_effector", [this](const std::shared_ptr<SetEndEffector::Request> req,
                                   std::shared_ptr<SetEndEffector::Response> res) { setFrame(*req, *res, nTe_); });
    camera_srv_ = create_service<SetEndEffector>(
        "camera/set_end_effector", [this](const std::shared_ptr<SetEndEffector::Request> req,
                                          std::shared_ptr<SetEndEffector::Response> res) { setFrame(*req, *res, nTc_); });
    null_srv_ = create_service<ActivateJointController>(
        "activate_joint_controller", [this](const std::shared_ptr<ActivateJointController::Request> req,
                                            std::shared_ptr<ActivateJointController::Response> res) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (req->activate)
          {
            if (static_cast<int>(req->qdes.position.size()) == n_)
              q_null_des_ = Eigen::Map<const Eigen::VectorXd>(req->qdes.position.data(), n_);
            else
              q_null_des_ = q_;
            null_start_ = now();
          }
          null_active_ = req->activate;
          res->success = true;
        });

    timer_ = create_wall_timer(std::chrono::duration<double>(sample_time_), [this]() { update(); });
    RCLCPP_INFO(get_logger(), "Robot kinematics ready: %d joints, flange '%s', sample time %.4f s", n_,
                flange.c_str(), sample_time_);
  }

private:
  void setSelection(const std::vector<bool>& v)
  {
    std::vector<int> rows;
    for (int i = 0; i < 6; ++i)
      if (v[i])
        rows.push_back(i);
    H_ = Eigen::MatrixXd::Zero(rows.size(), 6);
    for (size_t r = 0; r < rows.size(); ++r)
      H_(r, rows[r]) = 1.0;
  }

  void setFrame(const SetEndEffector::Request& req, SetEndEffector::Response& res, pinocchio::SE3& T)
  {
    if (!req.ee_name.empty())
    {
      res.success = false;
      res.message = "Named end effectors are not supported";
      return;
    }
    const auto& p = req.flange_pose_ee;
    Eigen::Quaterniond Q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    std::lock_guard<std::mutex> lock(mutex_);
    T = pinocchio::SE3(Q.normalized().toRotationMatrix(), Eigen::Vector3d(p.position.x, p.position.y, p.position.z));
    res.success = true;
    res.message = "End effector set";
  }

  /// Body Jacobian of the end effector (flange * nTe) at the current q, rows [linear; angular].
  Eigen::MatrixXd bodyJacobian()
  {
    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::Data::Matrix6x J_flange(6, n_);
    J_flange.setZero();
    pinocchio::getFrameJacobian(model_, data_, flange_id_, pinocchio::LOCAL, J_flange);
    return nTe_.inverse().toActionMatrix() * J_flange;
  }

  void twistCallback(const geometry_msgs::msg::TwistStamped& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Matrix<double, 6, 1> v;
    v << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y,
        msg.twist.angular.z;
    const Eigen::MatrixXd J = bodyJacobian();
    const Eigen::MatrixXd HJ = H_ * J;
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(HJ);
    Eigen::VectorXd qdot = cod.solve(H_ * v);

    if (null_active_)
    {
      const double t = (now() - null_start_).seconds();
      const double s = (t < 200.0) ? 1.0 - std::exp(-t / 10.0) : 1.0;
      Eigen::VectorXd qn = (null_gain_ * s * (q_null_des_ - q_)).cwiseMax(-null_saturation_).cwiseMin(null_saturation_);
      qdot += (Eigen::MatrixXd::Identity(n_, n_) - cod.pseudoInverse() * HJ) * qn;
    }

    for (int i = 0; i < n_; ++i)
    {
      if (std::abs(qdot[i]) > vel_limits_[i])
      {
        RCLCPP_ERROR(get_logger(), "Joint %d velocity %.3f exceeds the limit %.3f: stopping the integrator", i,
                     qdot[i], vel_limits_[i]);
        integrator_running_ = false;
        has_qdot_ = false;
        return;
      }
    }
    qdot_ = qdot;
    has_qdot_ = true;

    // end-effector twist in the end-effector frame
    const Eigen::Matrix<double, 6, 1> twist = J * qdot_;
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now();
    out.header.frame_id = frame_id_;
    out.twist.linear.x = twist[0];
    out.twist.linear.y = twist[1];
    out.twist.linear.z = twist[2];
    out.twist.angular.x = twist[3];
    out.twist.angular.y = twist[4];
    out.twist.angular.z = twist[5];
    twist_pub_->publish(out);
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (integrator_running_)
    {
      if (has_qdot_)
      {
        q_ += sample_time_ / 2.0 * (qdot_ + qdot_prev_);
        qdot_prev_ = qdot_;
      }
    }
    else if (has_command_)
    {
      q_ = q_command_;
    }

    for (int i = 0; i < n_; ++i)
    {
      if (q_[i] <= model_.lowerPositionLimit[i] || q_[i] >= model_.upperPositionLimit[i])
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Joint %d outside its limits (q = %.3f)", i, q_[i]);
        if (integrator_running_)
        {
          RCLCPP_ERROR(get_logger(), "Joint limits violated: stopping the integrator");
          integrator_running_ = false;
          has_qdot_ = false;
        }
      }
    }

    const auto stamp = now();
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = joint_names_;
    js.position.assign(q_.data(), q_.data() + n_);
    js.velocity.assign(n_, 0.0);
    if (integrator_running_ && has_qdot_)
      js.velocity.assign(qdot_.data(), qdot_.data() + n_);
    joint_pub_->publish(js);

    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacement(model_, data_, flange_id_);
    const pinocchio::SE3& bTn = data_.oMf[flange_id_];
    fkine_pub_->publish(toPose(bTn * nTe_, stamp));
    fkine_camera_pub_->publish(toPose(bTn * nTc_, stamp));
  }

  geometry_msgs::msg::PoseStamped toPose(const pinocchio::SE3& T, const rclcpp::Time& stamp) const
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = stamp;
    p.header.frame_id = frame_id_;
    p.pose.position.x = T.translation().x();
    p.pose.position.y = T.translation().y();
    p.pose.position.z = T.translation().z();
    const Eigen::Quaterniond Q(T.rotation());
    p.pose.orientation.w = Q.w();
    p.pose.orientation.x = Q.x();
    p.pose.orientation.y = Q.y();
    p.pose.orientation.z = Q.z();
    return p;
  }

  // model
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex flange_id_;
  int n_;
  std::vector<std::string> joint_names_;
  std::string frame_id_;
  pinocchio::SE3 nTe_, nTc_;

  // state
  std::mutex mutex_;
  double sample_time_;
  Eigen::VectorXd q_, q_command_, qdot_, qdot_prev_;
  bool integrator_running_ = false, has_qdot_ = false, has_command_ = false;
  std::vector<double> vel_limits_;
  Eigen::MatrixXd H_;
  bool null_active_ = false;
  double null_gain_, null_saturation_;
  Eigen::VectorXd q_null_des_;
  rclcpp::Time null_start_;

  // ROS
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_pub_, fkine_camera_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr integrator_srv_;
  rclcpp::Service<SetEndEffector>::SharedPtr ee_srv_, camera_srv_;
  rclcpp::Service<ActivateJointController>::SharedPtr null_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
