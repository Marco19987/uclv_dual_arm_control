
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "dual_arm_control_interfaces/srv/ekf_service.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "../include/geometry_helper.hpp"
#include "../ekf_ikfom/robot_spring_object_system_ikfom.hpp"

#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <random>
#include "../include/utils_read_yaml.hpp"

class SimulatorRobotsObject : public rclcpp::Node
{
public:
  SimulatorRobotsObject() : Node("simulator_robots_object")
  {
    // declare parameters
    this->declare_parameter<double>("sample_time", 0.004);
    this->get_parameter("sample_time", this->sample_time_);

    this->declare_parameter<double>("publishing_sample_time", 0.01);
    this->get_parameter("publishing_sample_time", this->publishing_sample_time);

    this->declare_parameter<double>("pose_measure_publishing_sample_time", 0.066666667);
    this->get_parameter("pose_measure_publishing_sample_time", this->pose_measure_publishing_sample_time_);

    this->declare_parameter<std::string>("robot_1_prefix", "robot1");
    this->get_parameter("robot_1_prefix", this->robot_1_prefix_);

    this->declare_parameter<std::string>("robot_2_prefix", "robot2");
    this->get_parameter("robot_2_prefix", this->robot_2_prefix_);

    this->declare_parameter<std::string>(
        "yaml_file_path", ament_index_cpp::get_package_share_directory("dual_arm_control") + "/config/config_sim.yaml");
    this->get_parameter("yaml_file_path", this->yaml_file_path_);

    this->declare_parameter<std::string>("object_name", "resin_block_1");
    this->get_parameter("object_name", this->object_name_);

    this->declare_parameter<bool>("sim_force_noise", false);
    this->get_parameter("sim_force_noise", this->sim_force_noise_);

    declare_ros_parameters();

    // initialize wrench publishers
    wrench_robot1_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>(this->robot_1_prefix_ + "/wrench", 1);
    wrench_robot2_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>(this->robot_2_prefix_ + "/wrench", 1);

    external_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("external_wrench_simulator", 1);

    internal_wrench_1_publisher =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("internal_wrench_1_simulator", 1);

    internal_wrench_2_publisher =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("internal_wrench_2_simulator", 1);

    // initialize wrench publishers
    fkine_robot1_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + this->robot_1_prefix_ + "/fkine_simulator_output", 1);
    fkine_robot2_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + this->robot_2_prefix_ + "/fkine_simulator_output", 1);

    // initialize pose publisher
    object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("object_pose", 1);
    object_twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("object_twist", 1);
    robot1_fkine_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(this->robot_1_prefix_ + "/fkine_simulator", 1);
    robot2_fkine_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(this->robot_2_prefix_ + "/fkine_simulator", 1);

    b2Tb1_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("b2Tb1_simulator", 1);
    bTb1_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("bTb1_simulator", 1);
    oTg1_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("oTg1_simulator", 1);
    oTg2_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("oTg2_simulator", 1);
    e1To_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("e1To_topic", 1);
    e2To_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("e2To_topic", 1);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("object_marker", 1);

    u_.e1ve1 << 0.0, 0.0, 0.0;     // robot1 linear velocity
    u_.e1omegae1 << 0.0, 0.0, 0.0; // robot1 angular velocity
    u_.e2ve2 << 0.0, 0.0, 0.0;     // robot2 linear velocity
    u_.e2omegae2 << 0.0, 0.0, 0.0; // robot2 angular velocity

    // initialize variables
    state_sim_.bTo.mat = Eigen::Matrix<double, 4, 4>::Identity();
    state_sim_.bTo.mat.block<3, 1>(0, 3) << 0.9, 0.0, 0.35; // object position in the base frame
    state_sim_.bTo.mat.block<3, 3>(0, 0) =
        Eigen::Quaterniond(0.0, 0.0, -0.7071, 0.7071).normalized().toRotationMatrix(); // object orientation in the
                                                                                       // base frame
    // otwisto
    state_sim_.ovo << 0.0, 0.0, 0.0;
    state_sim_.oomegao << 0.0, 0.0, 0.0;

    // Set b1Te1
    // state_sim_.b1Te1.mat = Eigen::Matrix<double, 4, 4>::Identity();
    // state_sim_.b1Te1.mat.block<3, 1>(0, 3) << 0.7715182967855756, 0.06723692220643047, 0.4450702143009003;
    // state_sim_.b1Te1.mat.block<3, 3>(0, 0) =
        // Eigen::Quaterniond(0.21567025560385847, 0.21284750027787291, 0.6915438799695846, -0.6557052268739666)
            // .normalized()
            // .toRotationMatrix();

    state_sim_.b1Te1.mat = Eigen::Matrix<double, 4, 4>::Identity();
    state_sim_.b1Te1.mat.block<3, 1>(0, 3) << 0.7715182967855756, 0.06723692220643047, 0.4450702143009003;
    state_sim_.b1Te1.mat.block<3, 3>(0, 0) = Eigen::Quaterniond(0, 0, -0.7071,
    0.7071).normalized().toRotationMatrix();

    // Set b2Te2
    // state_sim_.b2Te2.mat = Eigen::Matrix<double, 4, 4>::Identity();
    // state_sim_.b2Te2.mat.block<3, 1>(0, 3) << 0.0017974026641785355, 0.5281994881885076, 0.4340362840861629;
    // state_sim_.b2Te2.mat.block<3, 3>(0, 0) =
    //     Eigen::Quaterniond(0.6468801816062892, -0.27382021209535823, 0.6583836117457968, -0.27036926947718654)
    //         .normalized()
    //         .toRotationMatrix();

    state_sim_.b2Te2.mat = Eigen::Matrix<double, 4, 4>::Identity();
    state_sim_.b2Te2.mat.block<3, 1>(0, 3) << 0.0017974026641785355, 0.5281994881885076, 0.4340362840861629;
    state_sim_.b2Te2.mat.block<3, 3>(0, 0) =
        Eigen::Quaterniond(0.5000, 0.5000, -0.5000, -0.5000).normalized().toRotationMatrix();

    // Set b2Tb1
    state_sim_.b2Tb1.mat = Eigen::Matrix<double, 4, 4>::Identity();
    state_sim_.b2Tb1.mat.block<3, 1>(0, 3) << 0.0, 1.63, 0.0;
    state_sim_.b2Tb1.mat.block<3, 3>(0, 0) =
        Eigen::Quaterniond(0.707106781186548, 0, 0, -0.707106781186548).normalized().toRotationMatrix();

    // set h_v
    // state_sim_.f_v.setZero();
    // state_sim_.tau_v.setZero();
    selection_matrix_vector_robot1_.clear();
    selection_matrix_vector_robot2_.clear();
    selection_matrix_vector_robot1_ = {1, 1, 1, 1, 1, 1};
    selection_matrix_vector_robot2_ = {1, 1, 1, 1, 1, 1};
    update_selection_matrix();

    // read the yaml file
    read_yaml_file(yaml_file_path_, object_name_);

    // create the timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(sample_time_ * 1000)),
                                     std::bind(&SimulatorRobotsObject::timer_callback, this));
    // create the timer
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(publishing_sample_time * 1000)),
                                             std::bind(&SimulatorRobotsObject::publish_timer_callback, this));

    publish_pose_measure_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(pose_measure_publishing_sample_time_ * 1000)),
                                                          std::bind(&SimulatorRobotsObject::publish_pose_timer_callback, this));

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    cmd_twist_robot1_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/" + this->robot_1_prefix_ + "/command/fkine_twist_tactile_sensor_body_frame", qos,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        { this->cmd_twist_callback(msg, 0); });
    cmd_twist_robot2_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/" + this->robot_2_prefix_ + "/command/fkine_twist_tactile_sensor_body_frame", qos,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        { this->cmd_twist_callback(msg, 1); });

    // create service
    server_object_grasped_ = this->create_service<std_srvs::srv::SetBool>(
        "set_object_grasped", std::bind(&SimulatorRobotsObject::handle_grasp_service_request, this,
                                        std::placeholders::_1, std::placeholders::_2));

    // Create the service server
    server_ = this->create_service<dual_arm_control_interfaces::srv::EKFService>(
        "set_object_simulator",
        std::bind(&SimulatorRobotsObject::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void cmd_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg, int robot)
  {
    if (robot == 0)
    {
      u_.e1ve1(0) = msg->twist.linear.x;
      u_.e1ve1(1) = msg->twist.linear.y;
      u_.e1ve1(2) = msg->twist.linear.z;
      u_.e1omegae1(0) = msg->twist.angular.x;
      u_.e1omegae1(1) = msg->twist.angular.y;
      u_.e1omegae1(2) = msg->twist.angular.z;
    }

    if (robot == 1)
    {
      u_.e2ve2(0) = msg->twist.linear.x;
      u_.e2ve2(1) = msg->twist.linear.y;
      u_.e2ve2(2) = msg->twist.linear.z;
      u_.e2omegae2(0) = msg->twist.angular.x;
      u_.e2omegae2(1) = msg->twist.angular.y;
      u_.e2omegae2(2) = msg->twist.angular.z;
    }

    if (robot < 1)
    {
      last_twist_msg_robot1_ = msg;
    }
    else
    {
      last_twist_msg_robot2_ = msg;
    }
  }

  void publish_pose_timer_callback()
  {
    const int num_pose_measure = 2 * 6; // number of pose measurements
    process_params_.num_pose_measure_robot1 = num_frames_;
    process_params_.num_pose_measure_robot2 = num_frames_;
    MTK::MyManifold<double, num_pose_measure> h_(wrench_measure(), wrench_measure(), SEN(), SEN(),
                                                 std::vector<SEN>(num_pose_measure, SEN()));
    h_ = get_h<num_pose_measure>(state_sim_, process_params_);

    auto timestamp = this->now();

    // publish the measures

    for (int i = 0; i < process_params_.num_pose_measure_robot1; i++)
    {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = timestamp;
      pose_msg.pose.position.x = h_.object_pose[i].mat(0, 3);
      pose_msg.pose.position.y = h_.object_pose[i].mat(1, 3);
      pose_msg.pose.position.z = h_.object_pose[i].mat(2, 3);
      // Initialize orientation from rotation matrix
      Eigen::Matrix3d rot = h_.object_pose[i].mat.block<3, 3>(0, 0);
      Eigen::Quaterniond q(rot);
      pose_msg.pose.orientation.w = q.w();
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_publishers_[i]->publish(pose_msg);
    }
    for (int i = process_params_.num_pose_measure_robot1;
         i < process_params_.num_pose_measure_robot1 + process_params_.num_pose_measure_robot2; i++)
    {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = timestamp;
      pose_msg.pose.position.x = h_.object_pose[i].mat(0, 3);
      pose_msg.pose.position.y = h_.object_pose[i].mat(1, 3);
      pose_msg.pose.position.z = h_.object_pose[i].mat(2, 3);
      Eigen::Matrix3d rot = h_.object_pose[i].mat.block<3, 3>(0, 0);
      Eigen::Quaterniond q(rot);
      pose_msg.pose.orientation.w = q.w();
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_publishers_[i]->publish(pose_msg);
    }
  }

  void publish_timer_callback()
  {
    const int num_pose_measure = 2 * 6; // number of pose measurements
    process_params_.num_pose_measure_robot1 = num_frames_;
    process_params_.num_pose_measure_robot2 = num_frames_;
    MTK::MyManifold<double, num_pose_measure> h_(wrench_measure(), wrench_measure(), SEN(), SEN(),
                                                 std::vector<SEN>(num_pose_measure, SEN()));
    h_ = get_h<num_pose_measure>(state_sim_, process_params_);

    auto timestamp = this->now();

    // publish wrench
    Eigen::Vector<double, 6> force_noise;
    Eigen::Vector<double, 6> force_noise_var;

    force_noise.setZero();

    if (sim_force_noise_)
    { // Set the variance for the noise (example values, adjust as needed)
      force_noise_var << 0.0025, 0.0025, 0.000225, 1E-6, 1E-6, 9E-8;

      // Seed the random number generator
      static std::random_device rd;
      static std::mt19937 gen(rd());

      for (int i = 0; i < 6; ++i)
      {
        std::normal_distribution<double> dist(0.0, std::sqrt(force_noise_var(i)));
        force_noise(i) = dist(gen);
      }
    }

    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = timestamp;
    wrench_msg.wrench.force.x = h_.e1he1_e1.force(0) + force_noise(0);
    wrench_msg.wrench.force.y = h_.e1he1_e1.force(1) + force_noise(1);
    wrench_msg.wrench.force.z = h_.e1he1_e1.force(2) + force_noise(2);
    wrench_msg.wrench.torque.x = h_.e1he1_e1.torque(0) + force_noise(3);
    wrench_msg.wrench.torque.y = h_.e1he1_e1.torque(1) + force_noise(4);
    wrench_msg.wrench.torque.z = h_.e1he1_e1.torque(2) + force_noise(5);
    wrench_robot1_pub_->publish(wrench_msg);

    if (sim_force_noise_)
    { // Set the variance for the noise (example values, adjust as needed)
      force_noise_var << 0.0025, 0.0025, 0.000225, 1E-6, 1E-6, 9E-8;

      // Seed the random number generator
      static std::random_device rd;
      static std::mt19937 gen(rd());

      for (int i = 0; i < 6; ++i)
      {
        std::normal_distribution<double> dist(0.0, std::sqrt(force_noise_var(i)));
        force_noise(i) = dist(gen);
      }
    }

    wrench_msg.wrench.force.x = h_.e2he2_e2.force(0) + force_noise(0);
    wrench_msg.wrench.force.y = h_.e2he2_e2.force(1) + force_noise(1);
    wrench_msg.wrench.force.z = h_.e2he2_e2.force(2) + force_noise(2);
    wrench_msg.wrench.torque.x = h_.e2he2_e2.torque(0) + force_noise(3);
    wrench_msg.wrench.torque.y = h_.e2he2_e2.torque(1) + force_noise(4);
    wrench_msg.wrench.torque.z = h_.e2he2_e2.torque(2) + force_noise(5);
    wrench_robot2_pub_->publish(wrench_msg);

    // publish fkine
    geometry_msgs::msg::PoseStamped fkine_msg;
    fkine_msg.header.stamp = timestamp;
    fkine_msg.header.frame_id = "world"; // this->robot_1_prefix_;
    fkine_msg.pose.position.x = h_.b1_pose_e1.mat(0, 3);
    fkine_msg.pose.position.y = h_.b1_pose_e1.mat(1, 3);
    fkine_msg.pose.position.z = h_.b1_pose_e1.mat(2, 3);
    Eigen::Matrix3d rot = h_.b1_pose_e1.mat.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rot);
    fkine_msg.pose.orientation.w = q.w();
    fkine_msg.pose.orientation.x = q.x();
    fkine_msg.pose.orientation.y = q.y();
    fkine_msg.pose.orientation.z = q.z();
    fkine_robot1_pub_->publish(fkine_msg);

    fkine_msg.header.frame_id = "base_link"; // this->robot_2_prefix_;
    fkine_msg.pose.position.x = h_.b2_pose_e2.mat(0, 3);
    fkine_msg.pose.position.y = h_.b2_pose_e2.mat(1, 3);
    fkine_msg.pose.position.z = h_.b2_pose_e2.mat(2, 3);
    rot = h_.b2_pose_e2.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    fkine_msg.pose.orientation.w = q.w();
    fkine_msg.pose.orientation.x = q.x();
    fkine_msg.pose.orientation.y = q.y();
    fkine_msg.pose.orientation.z = q.z();
    fkine_robot2_pub_->publish(fkine_msg);

    // publish pose
    geometry_msgs::msg::PoseStamped object_pose_msg;
    object_pose_msg.header.stamp = timestamp;
    object_pose_msg.header.frame_id = "world";
    object_pose_msg.pose.position.x = state_sim_.bTo.mat(0, 3);
    object_pose_msg.pose.position.y = state_sim_.bTo.mat(1, 3);
    object_pose_msg.pose.position.z = state_sim_.bTo.mat(2, 3);
    rot = state_sim_.bTo.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    object_pose_msg.pose.orientation.w = q.w();
    object_pose_msg.pose.orientation.x = q.x();
    object_pose_msg.pose.orientation.y = q.y();
    object_pose_msg.pose.orientation.z = q.z();
    object_pose_publisher_->publish(object_pose_msg);

    // publish twist
    geometry_msgs::msg::TwistStamped object_twist_msg;
    object_twist_msg.header.stamp = timestamp;
    object_twist_msg.twist.linear.x = state_sim_.ovo(0);
    object_twist_msg.twist.linear.y = state_sim_.ovo(1);
    object_twist_msg.twist.linear.z = state_sim_.ovo(2);
    object_twist_msg.twist.angular.x = state_sim_.oomegao(0);
    object_twist_msg.twist.angular.y = state_sim_.oomegao(1);
    object_twist_msg.twist.angular.z = state_sim_.oomegao(2);
    object_twist_publisher_->publish(object_twist_msg);

    // publish the robot fkine
    geometry_msgs::msg::PoseStamped robot1_fkine_msg;
    robot1_fkine_msg.header.stamp = timestamp;
    robot1_fkine_msg.header.frame_id = "world"; // this->robot_1_prefix_;
    robot1_fkine_msg.pose.position.x = state_sim_.b1Te1.mat(0, 3);
    robot1_fkine_msg.pose.position.y = state_sim_.b1Te1.mat(1, 3);
    robot1_fkine_msg.pose.position.z = state_sim_.b1Te1.mat(2, 3);
    rot = state_sim_.b1Te1.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    robot1_fkine_msg.pose.orientation.w = q.w();
    robot1_fkine_msg.pose.orientation.x = q.x();
    robot1_fkine_msg.pose.orientation.y = q.y();
    robot1_fkine_msg.pose.orientation.z = q.z();
    robot1_fkine_pub_->publish(robot1_fkine_msg);

    geometry_msgs::msg::PoseStamped robot2_fkine_msg;
    robot2_fkine_msg.header.stamp = timestamp;
    robot2_fkine_msg.header.frame_id = "base_link"; // this->robot_2_prefix_;
    robot2_fkine_msg.pose.position.x = state_sim_.b2Te2.mat(0, 3);
    robot2_fkine_msg.pose.position.y = state_sim_.b2Te2.mat(1, 3);
    robot2_fkine_msg.pose.position.z = state_sim_.b2Te2.mat(2, 3);
    rot = state_sim_.b2Te2.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    robot2_fkine_msg.pose.orientation.w = q.w();
    robot2_fkine_msg.pose.orientation.x = q.x();
    robot2_fkine_msg.pose.orientation.y = q.y();
    robot2_fkine_msg.pose.orientation.z = q.z();
    robot2_fkine_pub_->publish(robot2_fkine_msg);

    // publish resulting wrench in the object frame
    // Eigen::Matrix<double, 6, 1> external_wrench;
    // int position_measure_vector = num_frames_ * 2 * 7;
    // robots_object_system_ptr_->get_object_wrench(y_.block<12, 1>(position_measure_vector, 0), external_wrench);

    // geometry_msgs::msg::WrenchStamped external_wrench_msg;
    // external_wrench_msg.header.stamp = timestamp;
    // external_wrench_msg.wrench.force.x = external_wrench(0);
    // external_wrench_msg.wrench.force.y = external_wrench(1);
    // external_wrench_msg.wrench.force.z = external_wrench(2);
    // external_wrench_msg.wrench.torque.x = external_wrench(3);
    // external_wrench_msg.wrench.torque.y = external_wrench(4);
    // external_wrench_msg.wrench.torque.z = external_wrench(5);
    // external_wrench_pub_->publish(external_wrench_msg);

    // publish internal and external wrench in the object frames (assume input constant)

    Eigen::Matrix<double, 12, 1> internal_wrench;
    internal_wrench.setZero();
    Eigen::Matrix<double, 6, 1> external_wrench;
    external_wrench.setZero();
    input_ikfom input;
    input = input_ikfom();

    compute_external_internal_wrench(state_sim_, input, process_params_, external_wrench, internal_wrench);

    // Publish external wrench
    geometry_msgs::msg::WrenchStamped external_wrench_msg;
    external_wrench_msg.header.stamp = timestamp;
    external_wrench_msg.wrench.force.x = external_wrench(0);
    external_wrench_msg.wrench.force.y = external_wrench(1);
    external_wrench_msg.wrench.force.z = external_wrench(2);
    external_wrench_msg.wrench.torque.x = external_wrench(3);
    external_wrench_msg.wrench.torque.y = external_wrench(4);
    external_wrench_msg.wrench.torque.z = external_wrench(5);
    external_wrench_pub_->publish(external_wrench_msg);

    // Publish internal wrench for robot 1
    geometry_msgs::msg::WrenchStamped internal_wrench_1_msg;
    internal_wrench_1_msg.header.stamp = timestamp;
    internal_wrench_1_msg.wrench.force.x = internal_wrench(0);
    internal_wrench_1_msg.wrench.force.y = internal_wrench(1);
    internal_wrench_1_msg.wrench.force.z = internal_wrench(2);
    internal_wrench_1_msg.wrench.torque.x = internal_wrench(3);
    internal_wrench_1_msg.wrench.torque.y = internal_wrench(4);
    internal_wrench_1_msg.wrench.torque.z = internal_wrench(5);
    internal_wrench_1_publisher->publish(internal_wrench_1_msg);

    // Publish internal wrench for robot 2
    geometry_msgs::msg::WrenchStamped internal_wrench_2_msg;
    internal_wrench_2_msg.header.stamp = timestamp;
    internal_wrench_2_msg.wrench.force.x = internal_wrench(6);
    internal_wrench_2_msg.wrench.force.y = internal_wrench(7);
    internal_wrench_2_msg.wrench.force.z = internal_wrench(8);
    internal_wrench_2_msg.wrench.torque.x = internal_wrench(9);
    internal_wrench_2_msg.wrench.torque.y = internal_wrench(10);
    internal_wrench_2_msg.wrench.torque.z = internal_wrench(11);
    internal_wrench_2_publisher->publish(internal_wrench_2_msg);

    // Publish b2Tb1 transform
    geometry_msgs::msg::PoseStamped b2Tb1_msg;
    b2Tb1_msg.header.stamp = timestamp;
    b2Tb1_msg.header.frame_id = "base_link";
    b2Tb1_msg.pose.position.x = state_sim_.b2Tb1.mat(0, 3);
    b2Tb1_msg.pose.position.y = state_sim_.b2Tb1.mat(1, 3);
    b2Tb1_msg.pose.position.z = state_sim_.b2Tb1.mat(2, 3);
    rot = state_sim_.b2Tb1.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    b2Tb1_msg.pose.orientation.w = q.w();
    b2Tb1_msg.pose.orientation.x = q.x();
    b2Tb1_msg.pose.orientation.y = q.y();
    b2Tb1_msg.pose.orientation.z = q.z();
    b2Tb1_pub_->publish(b2Tb1_msg);

    // Publish bTb1 transform
    geometry_msgs::msg::PoseStamped bTb1_msg;
    bTb1_msg.header.stamp = timestamp;
    bTb1_msg.header.frame_id = "world";
    bTb1_msg.pose.position.x = process_params_.bTb1.mat(0, 3);
    bTb1_msg.pose.position.y = process_params_.bTb1.mat(1, 3);
    bTb1_msg.pose.position.z = process_params_.bTb1.mat(2, 3);
    rot = process_params_.bTb1.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    bTb1_msg.pose.orientation.w = q.w();
    bTb1_msg.pose.orientation.x = q.x();
    bTb1_msg.pose.orientation.y = q.y();
    bTb1_msg.pose.orientation.z = q.z();
    bTb1_pub_->publish(bTb1_msg);

    // Publish oTg1 transform
    geometry_msgs::msg::PoseStamped oTg1_msg;
    oTg1_msg.header.stamp = timestamp;
    oTg1_msg.header.frame_id = "object_pose";
    oTg1_msg.pose.position.x = state_sim_.oTg1.mat(0, 3);
    oTg1_msg.pose.position.y = state_sim_.oTg1.mat(1, 3);
    oTg1_msg.pose.position.z = state_sim_.oTg1.mat(2, 3);
    rot = state_sim_.oTg1.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    oTg1_msg.pose.orientation.w = q.w();
    oTg1_msg.pose.orientation.x = q.x();
    oTg1_msg.pose.orientation.y = q.y();
    oTg1_msg.pose.orientation.z = q.z();
    oTg1_pub_->publish(oTg1_msg);

    // Publish oTg2 transform
    geometry_msgs::msg::PoseStamped oTg2_msg;
    oTg2_msg.header.stamp = timestamp;
    oTg2_msg.header.frame_id = "object_pose";
    oTg2_msg.pose.position.x = state_sim_.oTg2.mat(0, 3);
    oTg2_msg.pose.position.y = state_sim_.oTg2.mat(1, 3);
    oTg2_msg.pose.position.z = state_sim_.oTg2.mat(2, 3);
    rot = state_sim_.oTg2.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    oTg2_msg.pose.orientation.w = q.w();
    oTg2_msg.pose.orientation.x = q.x();
    oTg2_msg.pose.orientation.y = q.y();
    oTg2_msg.pose.orientation.z = q.z();
    oTg2_pub_->publish(oTg2_msg);

    // publish e1To and e2To
    Eigen::Matrix<double, 4, 4> e1To =
        state_sim_.b1Te1.mat.inverse() * process_params_.bTb1.mat.inverse() * state_sim_.bTo.mat;
    Eigen::Matrix<double, 4, 4> e2To =
        state_sim_.b2Te2.mat.inverse() * state_sim_.b2Tb1.mat * process_params_.bTb1.mat.inverse() * state_sim_.bTo.mat;

    geometry_msgs::msg::PoseStamped e1To_msg;
    e1To_msg.header.stamp = timestamp;
    e1To_msg.header.frame_id = "iiwa_link8";
    e1To_msg.pose.position.x = e1To(0, 3);
    e1To_msg.pose.position.y = e1To(1, 3);
    e1To_msg.pose.position.z = e1To(2, 3);
    rot = e1To.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    e1To_msg.pose.orientation.w = q.w();
    e1To_msg.pose.orientation.x = q.x();
    e1To_msg.pose.orientation.y = q.y();
    e1To_msg.pose.orientation.z = q.z();
    e1To_pub_->publish(e1To_msg);

    geometry_msgs::msg::PoseStamped e2To_msg;
    e2To_msg.header.stamp = timestamp;
    e2To_msg.header.frame_id = "yaskawa_tool0";
    e2To_msg.pose.position.x = e2To(0, 3);
    e2To_msg.pose.position.y = e2To(1, 3);
    e2To_msg.pose.position.z = e2To(2, 3);
    rot = e2To.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    e2To_msg.pose.orientation.w = q.w();
    e2To_msg.pose.orientation.x = q.x();
    e2To_msg.pose.orientation.y = q.y();
    e2To_msg.pose.orientation.z = q.z();
    e2To_pub_->publish(e2To_msg);

    // marker pub
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = timestamp;

    marker.ns = "object_shape";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::CUBE;

    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = state_sim_.bTo.mat(0, 3);
    marker.pose.position.y = state_sim_.bTo.mat(1, 3);
    marker.pose.position.z = state_sim_.bTo.mat(2, 3);
    rot = state_sim_.bTo.mat.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rot);
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();

    marker.scale.x = object_size_(0);
    marker.scale.y = object_size_(1);
    marker.scale.z = object_size_(2);

    marker.color.r = 0.5f;
    marker.color.g = 0.25f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5; // Don't forget to set the alpha!

    marker.lifetime = rclcpp::Duration::from_seconds(sample_time_);

    marker_pub_->publish(marker);
  }

  void timer_callback()
  {
    // x0_ = discretized_system_ptr_->get_state();

    // rotate the ionput from object frame to robot frame
    // Eigen::Quaterniond bQo(x0_(3), x0_(4), x0_(5), x0_(6));
    input_ikfom u_tmp;
    // u_tmp.block<3, 1>(0, 0) = bTb1_.block<3, 3>(0, 0).transpose() * bQo.toRotationMatrix() * u_.block<3, 1>(0, 0);

    // u_tmp.block<3, 1>(6, 0) = b1Tb2_.block<3, 3>(0, 0).transpose() * bTb1_.block<3, 3>(0, 0).transpose() *
    //                           bQo.toRotationMatrix() * (-  u_.block<3, 1>(0, 0) ); //* u_.block<3, 1>(6, 0);
    u_tmp = u_;

    Eigen::Matrix<double, state_dof, 1> f_sample_time = get_f(state_sim_, u_, process_params_) * sample_time_;
    state_sim_.boxplus(f_sample_time, 1);

    if (last_twist_msg_robot1_)
    {
      rclcpp::Time time_measure = last_twist_msg_robot1_->header.stamp;
      if (std::abs((time_measure - this->now()).seconds()) > 0.5)
      {
        u_.e1ve1.setZero();
        u_.e1omegae1.setZero();
      }
    }

    if (last_twist_msg_robot2_)
    {
      rclcpp::Time time_measure = last_twist_msg_robot2_->header.stamp;
      if (std::abs((time_measure - this->now()).seconds()) > 0.5)
      {
        u_.e2ve2.setZero();
        u_.e2omegae2.setZero();
      }
    }

    std::cout << "\nsample time: " << sample_time_ << "/n";
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "\nElapsed time: " << elapsed.count() << " s\n";
    std::cout << timer_->time_until_trigger().count() * 1E-6 << "\n";

    start = std::chrono::high_resolution_clock::now();
  }

  void read_yaml_file(const std::string &yaml_file_path, const std::string &object_name)
  {
    // read the yaml file
    RCLCPP_INFO(this->get_logger(), "Loading Configuration from %s\n", yaml_file_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Object Selected %s", object_name.c_str());
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    // check if the object is in the yaml file
    if (!config[object_name])
    {
      RCLCPP_ERROR(this->get_logger(), "Object %s not found in the YAML file", object_name.c_str());
      return;
    }

    // read gravity vector
    bg.setZero();
    std::vector<double> gravity = config["gravity_vector"].as<std::vector<double>>();
    bg << gravity[0], gravity[1], gravity[2];
    std::cout << "Gravity Vector: \n"
              << bg << std::endl;
    process_params_.bg = Eigen::Vector3d(0, 0, 0);

    // read b1Tb2
    Eigen::Matrix4d b1Tb2_;
    b1Tb2_.setIdentity();
    read_transform(config["b1Tb2"], b1Tb2_);
    std::cout << "b1Tb2: \n"
              << b1Tb2_ << std::endl;
    Eigen::Matrix<double, 4, 4> b2Tb1 = b1Tb2_.inverse();
    state_sim_.b2Tb1.mat = b2Tb1;

    // read bTb1
    Eigen::Matrix4d bTb1_;
    bTb1_.setIdentity();
    read_transform(config["bTb1"], bTb1_);
    std::cout << "bTb1: \n"
              << bTb1_ << std::endl;
    process_params_.bTb1.mat = bTb1_;

    YAML::Node object_node = config[object_name];

    // read viscous friction
    Eigen::Matrix<double, 6, 6> viscous_friction_matrix;
    read_matrix_6x6(config, process_params_.Bair, "viscous_friction");

    // read spring stiffness K_1
    read_matrix_6x6(config, K_1_matrix_, "K_1");

    // read spring stiffness K_2
    read_matrix_6x6(config, K_2_matrix_, "K_2");

    // read spring damping B_1
    read_matrix_6x6(config, B_1_matrix_, "B_1");

    // read spring damping B_2
    read_matrix_6x6(config, B_2_matrix_, "B_2");

    // read Bm
    read_matrix_6x6(object_node, process_params_.M, "inertia_matrix");

    // read object_size
    object_size_.setZero();
    std::vector<double> object_size = object_node["object_size"].as<std::vector<double>>();
    object_size_ << object_size[0], object_size[1], object_size[2];
    std::cout << "Object Size: \n"
              << object_size_ << std::endl;

    // read Kgrasp
    read_matrix_6x6(config, process_params_.K_grasp, "K_grasp");

    // read selection matrix
    // std::vector<bool> H_sel_1_diag = config["H_sel_1"].as<std::vector<bool>>();
    // std::vector<bool> H_sel_2_diag = config["H_sel_2"].as<std::vector<bool>>();

    // Eigen::MatrixXd Htmp1;
    // fill_selection_matrix(H_sel_1_diag, Htmp1);
    // Eigen::MatrixXd Htmp2;
    // fill_selection_matrix(H_sel_2_diag, Htmp2);

    // process_params_.H_sel.resize(Htmp1.rows() + Htmp2.rows(), Htmp1.cols() + Htmp2.cols());
    // process_params_.H_sel.setZero();
    // process_params_.H_sel.block(0, 0, Htmp1.rows(), Htmp1.cols()) = Htmp1;
    // process_params_.H_sel.block(Htmp1.rows(), Htmp1.cols(), Htmp2.rows(), Htmp2.cols()) = Htmp2;

    // std::cout << "\n H_sel matrix: \n" << process_params_.H_sel << std::endl;

    // // compute selection matrix negate
    // fill_selection_matrix_negate(H_sel_1_diag, Htmp1);
    // fill_selection_matrix_negate(H_sel_2_diag, Htmp2);

    // process_params_.H_sel_neg.resize(Htmp1.rows() + Htmp2.rows(), Htmp1.cols() + Htmp2.cols());
    // process_params_.H_sel_neg.setZero();
    // process_params_.H_sel_neg.block(0, 0, Htmp1.rows(), Htmp1.cols()) = Htmp1;
    // process_params_.H_sel_neg.block(Htmp1.rows(), Htmp1.cols(), Htmp2.rows(), Htmp2.cols()) = Htmp2;

    // std::cout << "\n H_sel matrix: \n" << process_params_.H_sel_neg << std::endl;

    // read names of frames published
    std::vector<std::string> frame_names;
    for (const auto &transformation : object_node["aruco_transforms"])
    {
      frame_names.push_back(transformation["name"].as<std::string>());
      RCLCPP_INFO(this->get_logger(), "Frame: %s", transformation["name"].as<std::string>().c_str());
    }

    // instantiate the subscribers to the pose topics
    int num_frames = frame_names.size();
    for (const auto &frame_name : frame_names)
    {
      pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          this->robot_1_prefix_ + "/" + object_name + "/" + frame_name + "/pose", 1));
    }

    for (const auto &frame_name : frame_names)
    {
      pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          this->robot_2_prefix_ + "/" + object_name + "/" + frame_name + "/pose", 1));
    }

    num_frames_ = num_frames;

    process_params_.K1 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.K2 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.B2 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.B2 = Eigen::Matrix<double, 6, 6>::Zero();
  }

  void handle_grasp_service_request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data) // object grasped
    {
      // enable contact in the model
      process_params_.K1 = K_1_matrix_;
      process_params_.B1 = B_1_matrix_;
      process_params_.K2 = K_2_matrix_;
      process_params_.B2 = B_2_matrix_;

      // set robot pose to the grasp pose

      Eigen::Matrix<double, 4, 4> oTg1;
      oTg1.setIdentity();
      oTg1 = state_sim_.bTo.mat.inverse() * process_params_.bTb1.mat * state_sim_.b1Te1.mat;
      std::cout << "oTg1: \n"
                << oTg1 << std::endl;
      Eigen::Matrix<double, 4, 4> oTg2;
      oTg2.setIdentity();
      oTg2 = state_sim_.bTo.mat.inverse() * process_params_.bTb1.mat * state_sim_.b2Tb1.mat.inverse() *
             state_sim_.b2Te2.mat;
      std::cout << "oTg2: \n"
                << oTg2 << std::endl;
      state_sim_.oTg1.mat = oTg1;
      state_sim_.oTg2.mat = oTg2;

      process_params_.bg = bg;
    }
    else
    {
      process_params_.K1 = Eigen::Matrix<double, 6, 6>::Zero();
      process_params_.B1 = Eigen::Matrix<double, 6, 6>::Zero();
      process_params_.K2 = Eigen::Matrix<double, 6, 6>::Zero();
      process_params_.B2 = Eigen::Matrix<double, 6, 6>::Zero();
      process_params_.bg = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
      state_sim_.ovo.setZero();
      state_sim_.oomegao.setZero();
    }

    response->success = true;
    response->message = request->data ? "object grasped True" : "object grasped False";
  }

  void handle_service_request(const std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Request> request,
                              std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request");

    timer_->cancel();

    // reset variables
    u_.e1ve1.setZero();
    u_.e2ve2.setZero();
    u_.e1omegae1.setZero();
    u_.e2omegae2.setZero();

    pose_publishers_.clear();

    state_sim_.ovo.setZero();
    state_sim_.oomegao.setZero();
    state_sim_.bTo.mat.setIdentity();
    state_sim_.b1Te1.mat.setIdentity();
    state_sim_.b2Te2.mat.setIdentity();
    state_sim_.b2Tb1.mat.setIdentity();
    state_sim_.oTg1.mat.setIdentity();
    state_sim_.oTg2.mat.setIdentity();
    // state_sim_.f_v.setZero();
    // state_sim_.tau_v.setZero();

    process_params_ = ProcessParams();

    // save initial state
    save_initial_state(request);

    // read yaml file
    selection_matrix_vector_robot1_.clear();
    selection_matrix_vector_robot2_.clear();
    selection_matrix_vector_robot1_ = {1, 1, 1, 1, 1, 1};
    selection_matrix_vector_robot2_ = {1, 1, 1, 1, 1, 1};
    update_selection_matrix();

    read_yaml_file(request->yaml_file_path.data.c_str(), request->object_name.data.c_str());
    process_params_.K1 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.B1 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.K2 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.B2 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.bg = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);

    timer_->reset();

    response->success = true;
  }

  void save_initial_state(const std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Request> request)
  {
    fill_T_from_pose_message(request->object_pose, state_sim_.bTo.mat);

    state_sim_.ovo << request->object_twist.twist.linear.x, request->object_twist.twist.linear.y,
        request->object_twist.twist.linear.z;
    state_sim_.oomegao << request->object_twist.twist.angular.x, request->object_twist.twist.angular.y,
        request->object_twist.twist.angular.z;

    fill_T_from_pose_message(request->fkine_robot1, state_sim_.b1Te1.mat);

    fill_T_from_pose_message(request->fkine_robot2, state_sim_.b2Te2.mat);

    fill_T_from_pose_message(request->robots_relative_transform, state_sim_.b2Tb1.mat);
  }
  void fill_T_from_pose_message(const geometry_msgs::msg::PoseStamped &pose_msg, Eigen::Matrix4d &T)
  {
    T.setIdentity();
    T.block<3, 1>(0, 3) << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z;
    T.block<3, 3>(0, 0) = Eigen::Quaterniond(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
                                             pose_msg.pose.orientation.y, pose_msg.pose.orientation.z)
                              .normalized()
                              .toRotationMatrix();

    std::cout << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z;
  }

  void declare_ros_parameters()
  {
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    cb_handles_.clear();
    // selection matrix
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "robot1_selection_matrix_vector";
      desc.description = "selection matrix. Pass a vector of 6 bool elements.";
      desc.additional_constraints = "";
      desc.read_only = false;
      desc.floating_point_range.clear();
      desc.integer_range.clear();
      this->declare_parameter(desc.name, std::vector<bool>({true, true, true, true, true, true}), desc);
      cb_handles_.insert(
          {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                {
             selection_matrix_vector_robot1_ = p.as_bool_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
             update_selection_matrix(); })});
    }

    // selection matrix
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "robot2_selection_matrix_vector";
      desc.description = "selection matrix. Pass a vector of 6 bool elements.";
      desc.additional_constraints = "";
      desc.read_only = false;
      desc.floating_point_range.clear();
      desc.integer_range.clear();
      this->declare_parameter(desc.name, std::vector<bool>({true, true, true, true, true, true}), desc);
      cb_handles_.insert(
          {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                {
             selection_matrix_vector_robot2_ = p.as_bool_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
             rclcpp::sleep_for(std::chrono::seconds(1));
             update_selection_matrix(); })});
    }
  }

  void update_selection_matrix()
  {
    std::cout << "DEBUG: update method" << std::endl;
    if (!selection_matrix_vector_robot1_.empty() && !selection_matrix_vector_robot2_.empty())
    {
      Eigen::MatrixXd Htmp1;
      fill_selection_matrix(selection_matrix_vector_robot1_, Htmp1);
      Eigen::MatrixXd Htmp2;
      fill_selection_matrix(selection_matrix_vector_robot2_, Htmp2);

      process_params_.H_sel.resize(Htmp1.rows() + Htmp2.rows(), Htmp1.cols() + Htmp2.cols());
      process_params_.H_sel.setZero();
      process_params_.H_sel.block(0, 0, Htmp1.rows(), Htmp1.cols()) = Htmp1;
      process_params_.H_sel.block(Htmp1.rows(), Htmp1.cols(), Htmp2.rows(), Htmp2.cols()) = Htmp2;

      std::cout << "H_sel matrix:\n"
                << process_params_.H_sel << std::endl;

      // compute selection matrix negate
      fill_selection_matrix_negate(selection_matrix_vector_robot1_, Htmp1);
      fill_selection_matrix_negate(selection_matrix_vector_robot2_, Htmp2);

      process_params_.H_sel_neg.resize(Htmp1.rows() + Htmp2.rows(), Htmp1.cols() + Htmp2.cols());
      process_params_.H_sel_neg.setZero();
      process_params_.H_sel_neg.block(0, 0, Htmp1.rows(), Htmp1.cols()) = Htmp1;
      process_params_.H_sel_neg.block(Htmp1.rows(), Htmp1.cols(), Htmp2.rows(), Htmp2.cols()) = Htmp2;

      std::cout << "H_sel_neg matrix:\n"
                << process_params_.H_sel_neg << std::endl;
    }
  }

  // server object grasped
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_object_grasped_;
  rclcpp::Service<dual_arm_control_interfaces::srv::EKFService>::SharedPtr server_;

  // subscribers to poseStamped
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers_;

  // state publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr object_twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot1_fkine_pub_; // output
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot2_fkine_pub_; // output
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr b2Tb1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr bTb1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr oTg1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr oTg2_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot1_pub_; // state variable
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot2_pub_; // state variable

  // publisher to WrenchStamped
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot2_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr external_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr internal_wrench_1_publisher;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr internal_wrench_2_publisher;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr e1To_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr e2To_pub_;

  // subscribers to WrenchStamped for external commands
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_twist_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_twist_robot2_sub_;

  geometry_msgs::msg::TwistStamped::SharedPtr last_twist_msg_robot1_;
  geometry_msgs::msg::TwistStamped::SharedPtr last_twist_msg_robot2_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> cb_handles_;

  // timer for ekf update
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr publish_pose_measure_timer_;

  // strings to attach at the topic name to subscribe
  std::string robot_1_prefix_;
  std::string robot_2_prefix_;

  // define systems for EKF

  state_ikfom state_sim_; // state vector (struct)
  input_ikfom u_;         // input vector (struct)

  double sample_time_;                         // sample time for the filter
  double publishing_sample_time;               // sample time for the publishing
  double pose_measure_publishing_sample_time_; // sample time for publishing the visual pose measures
  int num_frames_;                             // number of frames measuring the object
  std::string yaml_file_path_;
  bool sim_force_noise_ = false;

  // store spring parameters
  ProcessParams process_params_;
  Eigen::Matrix<double, 6, 6> K_1_matrix_;
  Eigen::Matrix<double, 6, 6> B_1_matrix_;
  Eigen::Matrix<double, 6, 6> B_2_matrix_;
  Eigen::Matrix<double, 6, 6> K_2_matrix_;

  Eigen::Matrix<double, 3, 1> bg;
  Eigen::Matrix<double, 3, 1> object_size_;

  std::string object_name_;

  std::chrono::_V2::system_clock::time_point start = std::chrono::high_resolution_clock::now();

  std::vector<bool> selection_matrix_vector_robot1_;
  std::vector<bool> selection_matrix_vector_robot2_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatorRobotsObject>());
  rclcpp::shutdown();
  return 0;
}