#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <eigen3/Eigen/Geometry>
#include "geometry_helper.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class InternalForceControlNode : public rclcpp::Node
{
public:
  InternalForceControlNode() : Node("internal_force_control_node"), force_control_active_(false)
  {
    // declare parameters
    this->declare_parameter("sample_time", 0.02);
    this->get_parameter("sample_time", sample_time_);

    this->declare_parameter("linear_vel_saturation", 0.01);
    this->get_parameter("linear_vel_saturation", linear_vel_saturation_);

    this->declare_parameter("angular_vel_saturation_", 0.1);
    this->get_parameter("angular_vel_saturation_", angular_vel_saturation_);

    this->declare_parameter("force_control_gain_diag_vector",
                            std::vector<double>({1e-1, 1e-1, 1e-1, 1e-2, 1e-2, 1e-2}));
    std::vector<double> force_control_gain_diag_vector;
    this->get_parameter("force_control_gain_diag_vector", force_control_gain_diag_vector);
    force_control_gain_matrix_.setZero();
    force_control_gain_matrix_.diagonal() << force_control_gain_diag_vector[0], force_control_gain_diag_vector[1],
        force_control_gain_diag_vector[2], force_control_gain_diag_vector[3], force_control_gain_diag_vector[4],
        force_control_gain_diag_vector[5];

    // declare subscribers and publishers
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    sub_internal_wrench = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "internal_wrench", qos, [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        {
          ee_h_ee_internal_wrench_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
              msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

          if (!force_control_active_)
            return;
            
          ee_twist_ee_ = -force_control_gain_matrix_ * ee_h_ee_internal_wrench_;
          publish_control(); 
        });

    pub_ee_twist_ee_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("fkine_twist_body_frame", qos);

    // create service
    service_force_ = this->create_service<std_srvs::srv::SetBool>(
        "activate_force_control", std::bind(&InternalForceControlNode::activateForceControlCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));

    // initialize variables
    ee_twist_ee_.setZero();
    ee_h_ee_internal_wrench_.setZero();
  }

private:
  void activateForceControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    force_control_active_ = request->data;
    if (!force_control_active_)
    {
      force_control_active_ = false;
      ee_twist_ee_.setZero();
    }

    response->success = true;
    response->message = force_control_active_ ? "force Control activated" : "force Control deactivated";
  }

  void publish_control()
  {
    // Saturate both twists
    for (int i = 0; i < 3; i++)
    {
      if (ee_twist_ee_(i) > linear_vel_saturation_)
        ee_twist_ee_(i) = linear_vel_saturation_;
      else if (ee_twist_ee_(i) < -linear_vel_saturation_)
        ee_twist_ee_(i) = -linear_vel_saturation_;
    }

    for (int i = 3; i < 6; i++)
    {
      if (ee_twist_ee_(i) > angular_vel_saturation_)
        ee_twist_ee_(i) = angular_vel_saturation_;
      else if (ee_twist_ee_(i) < -angular_vel_saturation_)
        ee_twist_ee_(i) = -angular_vel_saturation_;
    }

    // Publish twist
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.twist.linear.x = ee_twist_ee_(0);
    twist_msg.twist.linear.y = ee_twist_ee_(1);
    twist_msg.twist.linear.z = ee_twist_ee_(2);
    twist_msg.twist.angular.x = ee_twist_ee_(3);
    twist_msg.twist.angular.y = ee_twist_ee_(4);
    twist_msg.twist.angular.z = ee_twist_ee_(5);

    pub_ee_twist_ee_->publish(twist_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_ee_twist_ee_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_internal_wrench;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_force_;

  // vars
  Eigen::Matrix<double, 6, 1> ee_twist_ee_;             // twist end effector body frame
  Eigen::Matrix<double, 6, 1> ee_h_ee_internal_wrench_; // internal wrench of the robots in body frame

  // parameters
  double sample_time_; //! not used
  Eigen::Matrix<double, 6, 6> force_control_gain_matrix_;

  // other variables
  bool force_control_active_ = false;

  // saturate the absolute twist
  double linear_vel_saturation_;
  double angular_vel_saturation_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InternalForceControlNode>());
  rclcpp::shutdown();
  return 0;
}