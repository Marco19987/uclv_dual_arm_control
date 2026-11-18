#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <eigen3/Eigen/Geometry>
#include "geometry_helper.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "../ekf_ikfom/robot_spring_object_system_ikfom.hpp"

class ObjectPoseControlNode : public rclcpp::Node
{
public:
  ObjectPoseControlNode() : Node("pose_control_node"), control_active_(false)
  {
    // declare parameters
    this->declare_parameter("sample_time", 0.02);
    this->get_parameter("sample_time", sample_time_);

    // declare parameters
    this->declare_parameter("eeTo_is_identity", false);
    this->get_parameter("eeTo_is_identity", eeTo_is_identity_);

    this->declare_parameter("linear_vel_saturation", 0.01);
    this->get_parameter("linear_vel_saturation", linear_vel_saturation_);

    this->declare_parameter("angular_vel_saturation", 0.1);
    this->get_parameter("angular_vel_saturation", angular_vel_saturation_);

    this->declare_parameter("pose_control_gain_diag_vector",
                            std::vector<double>({ 1e-1, 1e-1, 1e-1, 1e-2, 1e-2, 1e-2 }));
    std::vector<double> pose_control_gain_diag_vector;
    this->get_parameter("pose_control_gain_diag_vector", pose_control_gain_diag_vector);
    pose_control_gain_matrix_.setZero();
    pose_control_gain_matrix_.diagonal() << pose_control_gain_diag_vector[0], pose_control_gain_diag_vector[1],
        pose_control_gain_diag_vector[2], pose_control_gain_diag_vector[3], pose_control_gain_diag_vector[4],
        pose_control_gain_diag_vector[5];

    // declare subscribers and publishers
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    sub_object_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "object_pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          this->convertPoseToMatrix(msg, this->bTo_);
          this->object_pose_read_ = true;

          if (control_active_ && eeTo_read_)
          {
            if (!desired_object_pose_read_)
              bTo_d_ = bTo_;

            Eigen::Matrix<double, 6, 6> adj_eeTo = adjoint_SE3(eeTo_.matrix());
            Eigen::Vector<double, 6> error_dot_K =
                pose_control_gain_matrix_ * SEN::log((bTo_.inverse() * bTo_d_).matrix());  // boxminus(bTo_d_,bTo_)
            error_twist = adj_eeTo * (error_dot_K);
            publish_control();
          }
        });

    sub_desired_object_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "desired_object_pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          this->convertPoseToMatrix(msg, this->bTo_d_);
          this->desired_object_pose_read_ = true;
        });

    if (!eeTo_is_identity_)
    {
      sub_eeTo_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "eeTo_topic", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->convertPoseToMatrix(msg, this->eeTo_);
            fkine_frame_ = msg->header.frame_id;
            this->eeTo_read_ = true;
          });
    }

    sub_desired_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "desired_twist", qos, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          this->b_twist_o_d_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, msg->twist.angular.x,
              msg->twist.angular.y, msg->twist.angular.z;
          this->desired_object_twist_read_ = true;

          if (!eeTo_read_)
          {
            return;
          }
          Eigen::Matrix<double, 6, 6> adj_eeTo = adjoint_SE3(eeTo_.matrix());
          forward_twist = adj_eeTo * get_Rbar(bTo_d_.rotation().transpose()) * b_twist_o_d_;

          if (!control_active_)
            publish_control();
        });

    pub_ee_twist_ee_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("fkine_twist_body_frame", qos);

    // create service
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "~/activate_control",
        std::bind(&ObjectPoseControlNode::activateControlCallback, this, std::placeholders::_1, std::placeholders::_2));


    // initialize variables
    eeTo_.setIdentity();
    bTo_.setIdentity();
    bTo_d_.setIdentity();
    ee_twist_ee.setZero();
    b_twist_o_d_.setZero();
    error_twist.setZero();
    forward_twist.setZero();
    eeTo_read_ = eeTo_is_identity_;
  }

private:
  void publish_control()
  {
    ee_twist_ee = forward_twist + error_twist;
    // forward_twist.setZero();
    // error_twist.setZero();
    // Saturate both twists
    for (int i = 0; i < 3; i++)
    {
      if (ee_twist_ee(i) > linear_vel_saturation_)
        ee_twist_ee(i) = linear_vel_saturation_;
      else if (ee_twist_ee(i) < -linear_vel_saturation_)
        ee_twist_ee(i) = -linear_vel_saturation_;
    }

    for (int i = 3; i < 6; i++)
    {
      if (ee_twist_ee(i) > angular_vel_saturation_)
        ee_twist_ee(i) = angular_vel_saturation_;
      else if (ee_twist_ee(i) < -angular_vel_saturation_)
        ee_twist_ee(i) = -angular_vel_saturation_;
    }

    // Publish twist
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = fkine_frame_;
    twist_msg.twist.linear.x = ee_twist_ee(0);
    twist_msg.twist.linear.y = ee_twist_ee(1);
    twist_msg.twist.linear.z = ee_twist_ee(2);
    twist_msg.twist.angular.x = ee_twist_ee(3);
    twist_msg.twist.angular.y = ee_twist_ee(4);
    twist_msg.twist.angular.z = ee_twist_ee(5);

    pub_ee_twist_ee_->publish(twist_msg);
  }

  void convertPoseToMatrix(const geometry_msgs::msg::PoseStamped::SharedPtr msg, Eigen::Isometry3d& isometry)
  {
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                         msg->pose.orientation.z);
    isometry = Eigen::Isometry3d::Identity();
    isometry.linear() = q.normalized().toRotationMatrix();
    isometry.translation() = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  void activateControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data && !control_active_)
    {
      this->desired_object_pose_read_ = false;
    }
    else
    {
      if (control_active_ && !request->data)
        reset_server();
    }
    control_active_ = request->data;

    response->success = true;
    response->message = control_active_ ? "Control activated" : "Control deactivated";
  }

  void reset_server()
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = fkine_frame_;
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    for (int i = 0; i < 10; i++)
    {
      pub_ee_twist_ee_->publish(twist_msg);
    }
    // timer_->cancel();
    object_pose_read_ = false;
    desired_object_pose_read_ = false;
    desired_object_twist_read_ = false;
    eeTo_read_ = eeTo_is_identity_;
    control_active_ = false;
    eeTo_.setIdentity();
    bTo_.setIdentity();
    // bTo_d_.setIdentity();
    ee_twist_ee.setZero();
    b_twist_o_d_.setZero();
    forward_twist.setZero();
    error_twist.setZero();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_object_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_desired_object_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_eeTo_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_desired_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_ee_twist_ee_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  // transformation matrices
  Eigen::Isometry3d eeTo_;  // relative transform object-end_effector, expressed in the end effector frame
  Eigen::Isometry3d bTo_;
  Eigen::Isometry3d bTo_d_;

  // twists
  Eigen::Matrix<double, 6, 1> ee_twist_ee;    // twist end effector body frame
  Eigen::Matrix<double, 6, 1> b_twist_o_d_;   // desired object twist in base frame
  Eigen::Matrix<double, 6, 1> forward_twist;  // forward twist
  Eigen::Matrix<double, 6, 1> error_twist;    // error term twist

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // parameters
  double sample_time_;
  Eigen::Matrix<double, 6, 6> pose_control_gain_matrix_;

  // other variables
  bool object_pose_read_ = false;
  bool desired_object_pose_read_ = false;
  bool desired_object_twist_read_ = false;
  bool eeTo_read_ = false;
  bool control_active_ = false;

  std::string fkine_frame_;

  // saturate the absolute twist
  double linear_vel_saturation_;
  double angular_vel_saturation_;

  bool eeTo_is_identity_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectPoseControlNode>());
  rclcpp::shutdown();
  return 0;
}