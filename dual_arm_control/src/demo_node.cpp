#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "dual_arm_control/trajectory_clients.hpp"

#include "dual_arm_control_interfaces/action/cartesian_trajectory.hpp"
#include "dual_arm_control_interfaces/srv/set_end_effector.hpp"

#include "dual_arm_control_interfaces/srv/ekf_service.hpp"

#include <std_srvs/srv/set_bool.hpp>

#include <rcl_interfaces/srv/set_parameters.hpp>

#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "../include/utils_read_yaml.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

#include "dual_arm_control_interfaces/srv/activate_joint_controller.hpp"

/*
    demo node for cooperative robots taks execution
    - use_internal_force_control: bool, use force control or not
    - use_object_pose_control: bool, use object pose control or not
*/

void eigen_matrix_to_pose_msg(const Eigen::Matrix<double, 4, 4>& T, geometry_msgs::msg::Pose& pose)
{
  pose.position.x = T(0, 3);
  pose.position.y = T(1, 3);
  pose.position.z = T(2, 3);
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
}

void pose_msg_to_eigen_matrix(const geometry_msgs::msg::Pose& pose, Eigen::Matrix<double, 4, 4>& T)
{
  T.setIdentity();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
}

// When true (ROS parameter "auto_continue"), the task does not wait for ENTER but proceeds
// automatically after "auto_continue_delay" seconds. Used to run the whole demo from a single launch file.
bool g_auto_continue = false;
double g_auto_continue_delay = 2.0;

void wait_for_enter()
{
  if (g_auto_continue)
  {
    std::cout << "Auto-continue in " << g_auto_continue_delay << " s..." << std::endl;
    rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(g_auto_continue_delay * 1e9)));
    return;
  }
  std::cout << "Press ENTER to continue..." << std::endl;
  // Wait for a single ENTER key press, ignore other keys and Ctrl+C
  std::string input;
  do
  {
    std::getline(std::cin, input);
    if (!std::cin)
    {
      // If input stream is closed (e.g., Ctrl+C), exit gracefully
      std::cout << "Input interrupted. Exiting..." << std::endl;
      rclcpp::shutdown();
      exit(0);
    }
  } while (!input.empty());
}
void wait_for_enter(const std::string s)
{
  std::cout << "\n" << s << "\n" << std::endl;
  wait_for_enter();
}
void fill_pose(geometry_msgs::msg::Pose& pose, double px, double py, double pz, double qw, double qx, double qy,
               double qz)
{
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;
  pose.orientation.w = qw;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
}
void fill_twist(geometry_msgs::msg::Twist& twist, double vx, double vy, double vz, double wx, double wy, double wz)
{
  twist.linear.x = vx;
  twist.linear.y = vy;
  twist.linear.z = vz;
  twist.angular.x = wx;
  twist.angular.y = wy;
  twist.angular.z = wz;
}

void print_pose_stamped(geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  // print heaer
  std::cout << "Header: " << std::endl;
  std::cout << "Frame ID: " << pose->header.frame_id << std::endl;
  std::cout << "Stamp: " << pose->header.stamp.sec << " " << pose->header.stamp.nanosec << std::endl;
  // print pose
  std::cout << "Pose: " << std::endl;
  std::cout << "Position: " << pose->pose.position.x << " " << pose->pose.position.y << " " << pose->pose.position.z
            << std::endl;
  std::cout << "Orientation: " << pose->pose.orientation.x << " " << pose->pose.orientation.y << " "
            << pose->pose.orientation.z << " " << pose->pose.orientation.w << std::endl;
}

void print_joint_positions(const std::vector<double>& q)
{
  for (const auto& q_i : q)
  {
    std::cout << q_i << " ";
  }
  std::cout << std::endl;
}

template <typename ServiceT>
auto call_service(const std::shared_ptr<rclcpp::Client<ServiceT>>& client,
                  typename ServiceT::Request::SharedPtr request, typename ServiceT::Response::SharedPtr response,
                  bool wait = true)

{
  std::cout << "Calling service" << std::endl;

  while (!client->wait_for_service(std::chrono::seconds(2)))
  {
    if (!rclcpp::ok())
    {
      std::cerr << "Interrupted while waiting for the service. Exiting." << std::endl;
      throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
    }
    std::cout << "Service not available, waiting again..." << std::endl;
  }

  auto result = client->async_send_request(request);

  if (!wait)
  {
    return result;
  }

  // Wait for the result using the executor if provided.
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(30);  // Set a timeout duration

  while (rclcpp::ok())
  {
    if (result.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
    {
      auto response = result.get();
      if (response)
      {
        std::cout << "Service call succeeded" << std::endl;
        // Process the result here
        return result;
      }
      else
      {
        std::cerr << "Failed to call service" << std::endl;
        throw std::runtime_error("Failed to call service");
      }
    }

    if (std::chrono::steady_clock::now() - start_time > timeout)
    {
      std::cerr << "Service call timed out" << std::endl;
      throw std::runtime_error("Service call timed out");
    }
  }
  throw std::runtime_error("Service call interrupted");
}

bool stringToBool(const std::string& str)
{
  return str == "true" || str == "1";
}

void set_activate_status(const std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>>& client, bool activate,
                         bool wait = true)
{
  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = activate;

  call_service(client, request, std_srvs::srv::SetBool::Response::SharedPtr(), wait);
}

int main(int argc, char** argv)
{
  if (argc < 8)
  {
    RCLCPP_ERROR(rclcpp::get_logger("cooperative_robots_demo"),
                 "Usage: demo_node <use_internal_force_control> <use_object_pose_control> <use_ekf> "
                 "<use_estimated_b1Tb2> <use_pivoting> <use_aruco> <simulation>");
    return 1;
  }

  bool use_internal_force_control = stringToBool(argv[1]);
  bool use_object_pose_control = stringToBool(argv[2]);
  bool use_ekf = stringToBool(argv[3]);
  bool use_estimated_b1Tb2 = stringToBool(argv[4]);
  bool use_pivoting = stringToBool(argv[5]);
  bool use_aruco = stringToBool(argv[6]);
  bool simulation = stringToBool(argv[7]);

  std::cout << "use_internal_force_control: " << use_internal_force_control << std::endl;
  std::cout << "use_object_pose_control: " << use_object_pose_control << std::endl;
  std::cout << "use_ekf: " << use_ekf << std::endl;
  std::cout << "use_estimated_b1Tb2: " << use_estimated_b1Tb2 << std::endl;
  std::cout << "use_pivoting: " << use_pivoting << std::endl;
  std::cout << "use_aruco: " << use_aruco << std::endl;
  std::cout << "simulation: " << simulation << std::endl;

  // This public release only contains the simulation setup: the ArUco perception pipeline and the
  // slipping/pivoting grasp controller of the real setup are not included.
  if (use_aruco || use_pivoting)
  {
    std::cout << "[WARN] use_aruco and use_pivoting require the real hardware pipeline, which is not part of this "
                 "release. They are ignored."
              << std::endl;
    use_aruco = false;
    use_pivoting = false;
  }

  // init ros
  rclcpp::init(argc, argv);
  // The task sequencer blocks on trajectories and services: when ROS is shut down (Ctrl+C, end of the launch)
  // stop the whole process right away instead of waiting for steps that can no longer complete.
  rclcpp::on_shutdown([]() {
    std::cout << "\nShutdown requested: stopping the task sequencer." << std::endl;
    std::_Exit(0);
  });
  auto node = std::make_shared<rclcpp::Node>("cooperative_robots_demo");
  g_auto_continue = node->declare_parameter<bool>("auto_continue", false);
  g_auto_continue_delay = node->declare_parameter<double>("auto_continue_delay", 2.0);
  std::cout << "auto_continue: " << g_auto_continue << " (delay " << g_auto_continue_delay << " s)" << std::endl;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // PARAMETERS ---------------------------------
  const std::string fkine_robot1_topic = "/robot1/fkine";
  const std::string fkine_robot2_topic = "/robot2/fkine";
  std::string joint_state_topic_robot1 = "/robot1/joint_states";
  std::string joint_state_topic_robot2 = "/robot2/joint_states";
  if (!simulation)
  {
    joint_state_topic_robot1 = "/lbr/joint_states";
    joint_state_topic_robot2 = "/motoman/joint_states";
  }
  const std::string obj_pose_topic = "/ekf/object_pose";
  const std::string filtered_b1Tb2_topic = "/ekf/b1Tb2_filtered";
  const std::string e1To_topic = "/robot1/eeTo_topic";
  const std::string e2To_topic = "/robot2/eeTo_topic";
  const std::string external_wrench_topic = "/ekf/external_wrench_estimate";

  const std::string package_share_directory = ament_index_cpp::get_package_share_directory("dual_arm_control");
  const std::string obj_yaml_path = package_share_directory + "/config/config.yaml";
  const std::string construction_config_yaml_path = package_share_directory +
                                                    "/config/"
                                                    "construction_configurations.yaml";
  const std::string config_sim_yaml_path = package_share_directory + "/config/config_sim.yaml";

  //---------------------------------------------

  // ROS objects --------------------------------
  int tmp_index = 0;
  auto ekf_client = node->create_client<dual_arm_control_interfaces::srv::EKFService>("ekf_service");
  ekf_client->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto ekf_client_object_grasped = node->create_client<std_srvs::srv::SetBool>("/ekf_set_object_grasped");
  ekf_client_object_grasped->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto aurco_pose_conversion_parameter_client1 =
      node->create_client<rcl_interfaces::srv::SetParameters>("/robot1/pose_conversion_server/set_parameters");
  if (use_aruco)
    aurco_pose_conversion_parameter_client1->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto aurco_pose_conversion_parameter_client2 =
      node->create_client<rcl_interfaces::srv::SetParameters>("/robot2/pose_conversion_server/set_parameters");
  if (use_aruco)
    aurco_pose_conversion_parameter_client2->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto activate_joint_integrator_client_robot1 =
      node->create_client<std_srvs::srv::SetBool>("/robot1/joint_integrator/set_running");
  activate_joint_integrator_client_robot1->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto activate_joint_integrator_client_robot2 =
      node->create_client<std_srvs::srv::SetBool>("/robot2/joint_integrator/set_running");
  activate_joint_integrator_client_robot2->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto actvate_internal_force_control_1_client =
      node->create_client<std_srvs::srv::SetBool>("/robot1/activate_force_control");
  actvate_internal_force_control_1_client->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto actvate_internal_force_control_2_client =
      node->create_client<std_srvs::srv::SetBool>("/robot2/activate_force_control");
  actvate_internal_force_control_2_client->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto activate_object_pose_control_1_client =
      node->create_client<std_srvs::srv::SetBool>("/robot1/pose_control_node/activate_control");
  activate_object_pose_control_1_client->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto activate_object_pose_control_2_client =
      node->create_client<std_srvs::srv::SetBool>("/robot2/pose_control_node/activate_control");
  activate_object_pose_control_2_client->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto set_end_effector_camera_client_robot1 =
      node->create_client<dual_arm_control_interfaces::srv::SetEndEffector>("/robot1/camera/set_end_effector");
  set_end_effector_camera_client_robot1->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto set_end_effector_camera_client_robot2 =
      node->create_client<dual_arm_control_interfaces::srv::SetEndEffector>("/robot2/camera/set_end_effector");
  set_end_effector_camera_client_robot2->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto set_end_effector_client_robot1 =
      node->create_client<dual_arm_control_interfaces::srv::SetEndEffector>("/robot1/set_end_effector");
  set_end_effector_client_robot1->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto set_end_effector_client_robot2 =
      node->create_client<dual_arm_control_interfaces::srv::SetEndEffector>("/robot2/set_end_effector");
  set_end_effector_client_robot2->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto parameters_client_clik_body_robot1 =
      node->create_client<rcl_interfaces::srv::SetParameters>("/robot1/robot_kinematics/set_parameters");
  parameters_client_clik_body_robot1->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto parameters_client_clik_body_robot2 =
      node->create_client<rcl_interfaces::srv::SetParameters>("/robot2/robot_kinematics/set_parameters");
  parameters_client_clik_body_robot2->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto parameters_client_ekf = node->create_client<rcl_interfaces::srv::SetParameters>("/ekf_server/set_parameters");
  parameters_client_ekf->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto simulator_client_set_object =
      node->create_client<dual_arm_control_interfaces::srv::EKFService>("set_object_simulator");
  if (simulation)
    simulator_client_set_object->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto simulator_client_object_grasped = node->create_client<std_srvs::srv::SetBool>("/set_object_grasped");
  if (simulation)
    simulator_client_object_grasped->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto parameters_client_simulator =
      node->create_client<rcl_interfaces::srv::SetParameters>("/simulator_robots_object/set_parameters");
  if (simulation)
    parameters_client_simulator->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto service_client_robot1_null_motion =
      node->create_client<dual_arm_control_interfaces::srv::ActivateJointController>("/robot1/activate_joint_controller");
  service_client_robot1_null_motion->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto service_client_robot2_null_motion =
      node->create_client<dual_arm_control_interfaces::srv::ActivateJointController>("/robot2/activate_joint_controller");
  service_client_robot2_null_motion->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto force_placement_service_client =
      node->create_client<std_srvs::srv::SetBool>("/force_placement_node/set_running");
  force_placement_service_client->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto activate_robot_pose_control_1_client =
      node->create_client<std_srvs::srv::SetBool>("/robot1/robot_pose_control/activate_control");
  activate_robot_pose_control_1_client->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  auto activate_robot_pose_control_2_client =
      node->create_client<std_srvs::srv::SetBool>("/robot2/robot_pose_control/activate_control");
  activate_robot_pose_control_2_client->wait_for_service();
  std::cout << ++tmp_index << std::endl;

  // Objects to store ----------------------------
  dual_arm_control::JointTrajectoryClient joint_client_robot1(node, "/robot1/generate_joint_trajectory");
  dual_arm_control::JointTrajectoryClient joint_client_robot2(node, "/robot2/generate_joint_trajectory");
  dual_arm_control::CartesianTrajectoryClient cartesian_client_robot1(node, "/robot1/generate_cartesian_trajectory");
  dual_arm_control::CartesianTrajectoryClient cartesian_client_robot2(node, "/robot2/generate_cartesian_trajectory");
  dual_arm_control::CartesianTrajectoryClient cartesian_client_object_traj(node, "/generate_cartesian_trajectory");

  //---------------------------------------------

  // READ YAML FILE WITH OBJECT INFORMATION
  RCLCPP_INFO(node->get_logger(), "Loading Configuration from %s\n", obj_yaml_path.c_str());
  YAML::Node obj_yaml = YAML::LoadFile(obj_yaml_path.c_str());

  // READ YAML FILE WITH TASK INFORMATION
  RCLCPP_INFO(node->get_logger(), "Loading Configuration from %s\n", construction_config_yaml_path.c_str());
  YAML::Node construction_config_yaml = YAML::LoadFile(construction_config_yaml_path.c_str());

  RCLCPP_INFO(node->get_logger(), "Loading Configuration from %s\n", config_sim_yaml_path.c_str());
  YAML::Node config_sim_yaml = YAML::LoadFile(config_sim_yaml_path.c_str());

  // read b1Tb2 simulator
  Eigen::Matrix<double, 4, 4> b1Tb2_simulator;
  b1Tb2_simulator.setIdentity();
  read_transform(config_sim_yaml["b1Tb2"], b1Tb2_simulator);
  std::cout << "b1Tb2_simulator: \n" << b1Tb2_simulator << std::endl;

  // read b1Tb2
  Eigen::Matrix<double, 4, 4> b1Tb2;
  b1Tb2.setIdentity();
  read_transform(obj_yaml["b1Tb2"], b1Tb2);
  std::cout << "b1Tb2: \n" << b1Tb2 << std::endl;

  // read bTb1
  Eigen::Matrix<double, 4, 4> bTb1;
  bTb1.setIdentity();
  read_transform(obj_yaml["bTb1"], bTb1);
  std::cout << "bTb1: \n" << bTb1 << std::endl;

  // read camera wrt last link
  Eigen::Matrix<double, 4, 4> iiwa8_T_camera;
  iiwa8_T_camera.setIdentity();
  read_transform(construction_config_yaml["iiwa8_T_camera"], iiwa8_T_camera);
  std::cout << "iiwa8_T_camera: \n" << iiwa8_T_camera << std::endl;

  // read camera wrt last link
  Eigen::Matrix<double, 4, 4> yaskawa_tool0_T_camera;
  yaskawa_tool0_T_camera.setIdentity();
  read_transform(construction_config_yaml["yaskawa_tool0_T_camera"], yaskawa_tool0_T_camera);
  std::cout << "yaskawa_tool0_T_camera: \n" << yaskawa_tool0_T_camera << std::endl;

  // set end effector camera 1
  dual_arm_control_interfaces::srv::SetEndEffector::Request::SharedPtr request_end_effector =
      std::make_shared<dual_arm_control_interfaces::srv::SetEndEffector::Request>();
  eigen_matrix_to_pose_msg(iiwa8_T_camera, request_end_effector->flange_pose_ee);

  call_service(set_end_effector_camera_client_robot1, request_end_effector,
               dual_arm_control_interfaces::srv::SetEndEffector::Response::SharedPtr());

  // set end effector camera 2
  eigen_matrix_to_pose_msg(yaskawa_tool0_T_camera, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_camera_client_robot2, request_end_effector,
               dual_arm_control_interfaces::srv::SetEndEffector::Response::SharedPtr());

  // set tactile sensors end effector
  Eigen::Matrix<double, 4, 4> iiwa8_T_tactile;
  iiwa8_T_tactile.setIdentity();
  read_transform(construction_config_yaml["iiwa8_T_tactile"], iiwa8_T_tactile);
  std::cout << "iiwa8_T_tactile: \n" << iiwa8_T_tactile << std::endl;

  // read tactile sensor wrt the pre-pivoting joint
  Eigen::Matrix<double, 4, 4> yaskawa_tool0_T_tactile;
  yaskawa_tool0_T_tactile.setIdentity();
  read_transform(construction_config_yaml["yaskawa_tool0_T_tactile"], yaskawa_tool0_T_tactile);
  std::cout << "yaskawa_tool0_T_tactile: \n" << yaskawa_tool0_T_tactile << std::endl;

  // set end effector robot1
  eigen_matrix_to_pose_msg(iiwa8_T_tactile, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_client_robot1, request_end_effector,
               dual_arm_control_interfaces::srv::SetEndEffector::Response::SharedPtr());

  // set end effector camera 2
  eigen_matrix_to_pose_msg(yaskawa_tool0_T_tactile, request_end_effector->flange_pose_ee);
  call_service(set_end_effector_client_robot2, request_end_effector,
               dual_arm_control_interfaces::srv::SetEndEffector::Response::SharedPtr());

  //-----------------------------------------------

  // deactivate joints integrators - security check
  set_activate_status(activate_joint_integrator_client_robot1, false);
  set_activate_status(activate_joint_integrator_client_robot2, false);

  Eigen::Matrix<double, 4, 4> oTg1;
  oTg1.setIdentity();
  Eigen::Matrix<double, 4, 4> oTg2;
  oTg2.setIdentity();

  bool ekf_filter_initialized = false;

  // Lambda to handle null space activation for both robots
  auto handle_null_space_activation =
      [&](const YAML::Node& action,
          std::shared_ptr<rclcpp::Client<dual_arm_control_interfaces::srv::ActivateJointController>>& client,
          const std::string& q_null_activate_key, const std::string& q_null_key) {
        if (action[q_null_activate_key])
        {
          if (action[q_null_key] && action[q_null_activate_key].as<bool>())
          {
            auto request = std::make_shared<dual_arm_control_interfaces::srv::ActivateJointController::Request>();
            auto response = std::make_shared<dual_arm_control_interfaces::srv::ActivateJointController::Response>();
            request->activate = true;
            request->qdes.header.stamp = node->now();
            request->qdes.position = action[q_null_key].as<std::vector<double>>();
            call_service(client, request, response, true);
          }
          else if (!action[q_null_activate_key].as<bool>())
          {
            auto request = std::make_shared<dual_arm_control_interfaces::srv::ActivateJointController::Request>();
            auto response = std::make_shared<dual_arm_control_interfaces::srv::ActivateJointController::Response>();
            request->activate = false;
            call_service(client, request, response, true);
          }
        }
      };

  double scale_time = 1.0;
  if (simulation && construction_config_yaml["scale_time_simulation"])
    scale_time = construction_config_yaml["scale_time_simulation"].as<double>();

  // TASK -   ----------------------------------
  for (const auto& task : construction_config_yaml["task_sequence"])
  {
    // READ YAML FILE WITH TASK INFORMATION
    const std::string task_yaml_path = package_share_directory + "/" + task["yaml_file"].as<std::string>();
    RCLCPP_INFO(node->get_logger(), "Loading Configuration from %s\n", task_yaml_path.c_str());
    std::cout << "Task: " << task["description"].as<std::string>() << std::endl;
    YAML::Node task_yaml = YAML::LoadFile(task_yaml_path.c_str());
    wait_for_enter();

    for (const auto& action : task_yaml["task"])
    {
      std::cout << "Executing:" << action["name"].as<std::string>() << "\n";

      if (action["wait_for_enter"])
        wait_for_enter();

      // Set selection matrix
      if (action["H_sel_robot1"])
      {
        // wait_for_enter("set selection matrix robot1");
        rcl_interfaces::srv::SetParameters::Request::SharedPtr set_parameter_request =
            std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        std::vector<bool> H_sel = action["H_sel_robot1"].as<std::vector<bool>>();
        set_parameter_request->parameters.push_back(
            rclcpp::Parameter("selection_matrix_vector", H_sel).to_parameter_msg());
        call_service(parameters_client_clik_body_robot1, set_parameter_request,
                     rcl_interfaces::srv::SetParameters::Response::SharedPtr());

        // ! set also to EKF and simulator
        set_parameter_request->parameters.clear();
        set_parameter_request->parameters.push_back(
            rclcpp::Parameter("robot1_selection_matrix_vector", H_sel).to_parameter_msg());
        call_service(parameters_client_ekf, set_parameter_request,
                     rcl_interfaces::srv::SetParameters::Response::SharedPtr());

        if (simulation)
        {
          call_service(parameters_client_simulator, set_parameter_request,
                       rcl_interfaces::srv::SetParameters::Response::SharedPtr());
        }
      }
      if (action["H_sel_robot2"])
      {
        // wait_for_enter("set selection matrix robot2");
        rcl_interfaces::srv::SetParameters::Request::SharedPtr set_parameter_request =
            std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        std::vector<bool> H_sel = action["H_sel_robot2"].as<std::vector<bool>>();
        set_parameter_request->parameters.push_back(
            rclcpp::Parameter("selection_matrix_vector", H_sel).to_parameter_msg());
        call_service(parameters_client_clik_body_robot2, set_parameter_request,
                     rcl_interfaces::srv::SetParameters::Response::SharedPtr());

        // ! set also to EKF and simulator
        set_parameter_request->parameters.clear();
        set_parameter_request->parameters.push_back(
            rclcpp::Parameter("robot2_selection_matrix_vector", H_sel).to_parameter_msg());
        call_service(parameters_client_ekf, set_parameter_request,
                     rcl_interfaces::srv::SetParameters::Response::SharedPtr());

        if (simulation)
        {
          call_service(parameters_client_simulator, set_parameter_request,
                       rcl_interfaces::srv::SetParameters::Response::SharedPtr());
        }
      }

      // Initialize EKF
      if (action["command"])
      {
        if (action["command"].as<std::string>() == "init_ekf" && action["object_name"])
        {
          // enable simulator
          if (simulation)
          {
            // call simulator service
            auto request = std::make_shared<dual_arm_control_interfaces::srv::EKFService::Request>();
            request->object_name.data = action["object_name"].as<std::string>();
            request->yaml_file_path.data = config_sim_yaml_path;
            // read final_pose bTo_final
            Eigen::Matrix<double, 4, 4> bTo_initial;
            bTo_initial.setIdentity();
            read_transform(action["bTo_initial"], bTo_initial);
            eigen_matrix_to_pose_msg(bTo_initial, request->object_pose.pose);
            fill_twist(request->object_twist.twist, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            // read fkine robot1
            {
              auto pose_ptr = dual_arm_control::waitForMessage<geometry_msgs::msg::PoseStamped>(fkine_robot1_topic, node);
              auto pose = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(pose_ptr);
              request->fkine_robot1.pose = pose->pose;
            }
            // read fkine robot2
            {
              auto pose_ptr = dual_arm_control::waitForMessage<geometry_msgs::msg::PoseStamped>(fkine_robot2_topic, node);
              auto pose = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(pose_ptr);
              request->fkine_robot2.pose = pose->pose;
            }

            eigen_matrix_to_pose_msg(b1Tb2_simulator, request->robots_relative_transform.pose);

            std::cout << "Calling simulator service" << std::endl;
            call_service(simulator_client_set_object, request,
                         dual_arm_control_interfaces::srv::EKFService::Response::SharedPtr());
            // wait_for_enter("Waiting for simulator to set the object");
          }
          // 2. CALL EKF SERVICE TO GET OBJECT POSE
          if (use_ekf)
          {
            // call EKF service
            auto request = std::make_shared<dual_arm_control_interfaces::srv::EKFService::Request>();
            request->object_name.data = action["object_name"].as<std::string>();
            request->yaml_file_path.data = obj_yaml_path;
            fill_pose(request->object_pose.pose, 0.9, 0.0, 0.35, 0.0, 0.0, -0.707106781186547, 0.707106781186547);
            fill_twist(request->object_twist.twist, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            Eigen::Matrix<double, 7, 1> x0_;
            x0_.setZero();
            x0_(0) = 0.7715182967855756;
            x0_(1) = 0.06723692220643047;
            x0_(2) = 0.4450702143009003;
            x0_(3) = 0.21567025560385847;
            x0_(4) = 0.21284750027787291;
            x0_(5) = 0.6915438799695846;
            x0_(6) = -0.6557052268739666;
            fill_pose(request->fkine_robot1.pose, x0_(0), x0_(1), x0_(2), x0_(3), x0_(4), x0_(5), x0_(6));
            x0_(0) = 0.0017974026641785355;
            x0_(1) = 0.5281994881885076;
            x0_(2) = 0.4340362840861629;
            x0_(3) = 0.6468801816062892;
            x0_(4) = -0.27382021209535823;
            x0_(5) = 0.6583836117457968;
            x0_(6) = -0.27036926947718654;

            fill_pose(request->fkine_robot2.pose, x0_(0), x0_(1), x0_(2), x0_(3), x0_(4), x0_(5), x0_(6));
            fill_pose(request->robots_relative_transform.pose, 0.0, 1.63, 0.0, 0.707, 0.0, 0.0, -0.707);

            std::cout << "Calling EKF service" << std::endl;
            call_service(ekf_client, request, dual_arm_control_interfaces::srv::EKFService::Response::SharedPtr());

            ekf_filter_initialized = true;

            wait_for_enter("Waiting for robot calibration");

            // if use_estimated_b1Tb2 is true, then read the estimated b1Tb2 from the topic
            if (use_estimated_b1Tb2)
            {
              // read estimated b1Tb2 from the topic
              auto estimated_b1Tb2_ptr =
                  dual_arm_control::waitForMessage<geometry_msgs::msg::PoseStamped>(filtered_b1Tb2_topic, node);
              auto estimated_b1Tb2 = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(estimated_b1Tb2_ptr);

              pose_msg_to_eigen_matrix(estimated_b1Tb2->pose, b1Tb2);
              std::cout << "Estimated b1Tb2: \n" << b1Tb2 << std::endl;
            }
          }
        }

        // read grasp poses
        if (action["command"].as<std::string>() == "read_grasp_poses")
        {
          if (action["oTg1"])
          {
            read_transform(action["oTg1"], oTg1);
            std::cout << "oTg1: \n" << oTg1 << std::endl;
          }
          if (action["oTg2"])
          {
            read_transform(action["oTg2"], oTg2);
            std::cout << "oTg2: \n" << oTg2 << std::endl;
          }
        }

        // read grasp poses
        if (action["command"].as<std::string>() == "activate_joint_integrators")
        {
          if (action["activate_integrator_robot1"])
          {
            set_activate_status(activate_joint_integrator_client_robot1,
                                action["activate_integrator_robot1"].as<bool>());
          }
          if (action["activate_integrator_robot2"])
          {
            set_activate_status(activate_joint_integrator_client_robot2,
                                action["activate_integrator_robot2"].as<bool>());
          }
        }
      }

      // movement robot1
      if (action["q_robot1"])
      {
        std::cout << "Joint movement"
                  << "\n";
        print_joint_positions(action["q_robot1"].as<std::vector<double>>());
        std::cout << "duration: " << action["duration_robot1"].as<double>() << "\n";
        set_activate_status(activate_joint_integrator_client_robot1, false);  // security stop integrators
        bool wait_traj = true;
        if (action["wait_robot1"])
          wait_traj = action["wait_robot1"].as<bool>();
        joint_client_robot1.goTo(joint_state_topic_robot1, action["q_robot1"].as<std::vector<double>>(),
                                 rclcpp::Duration::from_seconds(scale_time * action["duration_robot1"].as<double>()),
                                 rclcpp::Time(0), wait_traj);
      }
      else if (action["pose_robot1"] || action["offset_robot1"])  // if the command is movec - single cartesian
                                                                  // movement - and ther is the pose item, then move
                                                                  // the robot to that pose. If pose is absent,
                                                                  // compute the end effector pose using the object
                                                                  // estimate and the oTg grasp frames
      {
        std::cout << "Cartesian movement"
                  << "\n";

        // activate robot pose control
        Eigen::Matrix<double, 4, 4> biTei;
        biTei.setIdentity();

        if (action["pose_robot1"])
          read_transform(action["pose_robot1"], biTei);
        else
        {
          if (ekf_filter_initialized)
          {
            // read object pose from the topic
            auto object_pose_ptr = dual_arm_control::waitForMessage<geometry_msgs::msg::PoseStamped>(obj_pose_topic, node);
            auto object_pose = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(object_pose_ptr);

            Eigen::Matrix<double, 4, 4> bTo = Eigen::Matrix<double, 4, 4>::Identity();
            pose_msg_to_eigen_matrix(object_pose->pose, bTo);
            std::cout << "Object pose: \n" << bTo << std::endl;
            Eigen::Matrix<double, 4, 4> giTgi_pregrasp = Eigen::Matrix<double, 4, 4>::Identity();
            if (action["offset_robot1"])
            {
              std::vector<double> offset_vec = action["offset_robot1"].as<std::vector<double>>();
              if (offset_vec.size() == 3)
              {
                giTgi_pregrasp.block<3, 1>(0, 3) = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]);
              }
              else
              {
                throw std::runtime_error("offset_robot1 must be a vector of size 3");
              }
            }
            else
            {
              continue;
            }

            biTei = bTb1.inverse() * bTo * oTg1 * giTgi_pregrasp;
          }
        }
        std::cout << "biTei: \n" << biTei << std::endl;
        std::cout << "duration_robot1: " << action["duration_robot1"].as<double>() << "\n";

        bool wait_traj = true;
        if (action["wait_robot1"])
          wait_traj = action["wait_robot1"].as<bool>();
        double offset_start_time = 0;
        if (action["offset_start_time_robot1"])
          offset_start_time = action["offset_start_time_robot1"].as<double>();
        auto start_time = node->now() + rclcpp::Duration::from_seconds(offset_start_time);

        // activate joint integrators
        geometry_msgs::msg::Pose target_pose;
        eigen_matrix_to_pose_msg(biTei, target_pose);

        cartesian_client_robot1.goTo(
            fkine_robot1_topic, target_pose,
            rclcpp::Duration::from_seconds(scale_time * action["duration_robot1"].as<double>()), start_time, wait_traj);
      }
      else if (action["offset_robot1_EE"])
      {
        Eigen::Matrix<double, 4, 4> fkine1_T_fkine1postgrasp;
        fkine1_T_fkine1postgrasp.setIdentity();

        std::vector<double> offset_vec = action["offset_robot1_EE"].as<std::vector<double>>();
        if (offset_vec.size() == 3)
        {
          fkine1_T_fkine1postgrasp.block<3, 1>(0, 3) = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]);
        }
        else
        {
          throw std::runtime_error("offset_robot1_EE must be a vector of size 3");
        }

        geometry_msgs::msg::Pose target_pose;
        eigen_matrix_to_pose_msg(fkine1_T_fkine1postgrasp, target_pose);

        bool wait_traj = true;
        if (action["wait_robot1"])
          wait_traj = action["wait_robot1"].as<bool>();

        cartesian_client_robot1.goTo_EE(
            fkine_robot1_topic, target_pose,
            rclcpp::Duration::from_seconds(scale_time * action["duration_robot1"].as<double>()), rclcpp::Time(0),
            wait_traj);
      }

      // movement robot2
      if (action["q_robot2"])
      {
        std::cout << "Joint movement"
                  << "\n";
        print_joint_positions(action["q_robot2"].as<std::vector<double>>());
        std::cout << "duration: " << action["duration_robot2"].as<double>() << "\n";
        bool wait_traj = true;
        if (action["wait_robot2"])
          wait_traj = action["wait_robot2"].as<bool>();
        set_activate_status(activate_joint_integrator_client_robot2, false);  // security stop integrators
        joint_client_robot2.goTo(joint_state_topic_robot2, action["q_robot2"].as<std::vector<double>>(),
                                 rclcpp::Duration::from_seconds(scale_time * action["duration_robot2"].as<double>()),
                                 rclcpp::Time(0), wait_traj);
      }
      else if (action["pose_robot2"] || action["offset_robot2"])
      {
        std::cout << "Cartesian movement"
                  << "\n";
        // activate robot pose control
        Eigen::Matrix<double, 4, 4> biTei;
        biTei.setIdentity();

        if (action["pose_robot2"])
          read_transform(action["pose_robot2"], biTei);
        else
        {
          if (ekf_filter_initialized)
          {
            // read object pose from the topic
            auto object_pose_ptr = dual_arm_control::waitForMessage<geometry_msgs::msg::PoseStamped>(obj_pose_topic, node);
            auto object_pose = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(object_pose_ptr);

            Eigen::Matrix<double, 4, 4> bTo = Eigen::Matrix<double, 4, 4>::Identity();
            pose_msg_to_eigen_matrix(object_pose->pose, bTo);
            std::cout << "Object pose: \n" << bTo << std::endl;
            Eigen::Matrix<double, 4, 4> giTgi_pregrasp = Eigen::Matrix<double, 4, 4>::Identity();
            if (action["offset_robot2"])
            {
              std::vector<double> offset_vec = action["offset_robot2"].as<std::vector<double>>();
              if (offset_vec.size() == 3)
              {
                giTgi_pregrasp.block<3, 1>(0, 3) = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]);
              }
              else
              {
                throw std::runtime_error("offset_robot2 must be a vector of size 3");
              }
            }
            biTei = b1Tb2.inverse() * bTb1.inverse() * bTo * oTg2 * giTgi_pregrasp;
          }
        }
        std::cout << "biTei: \n" << biTei << std::endl;
        std::cout << "duration_robot2: " << action["duration_robot2"].as<double>() << "\n";

        bool wait_traj = true;
        if (action["wait_robot2"])
          wait_traj = action["wait_robot2"].as<bool>();
        double offset_start_time = 0;
        if (action["offset_start_time_robot2"])
          offset_start_time = action["offset_start_time_robot2"].as<double>();
        auto start_time = node->now() + rclcpp::Duration::from_seconds(offset_start_time);

        // activate joint integrators
        geometry_msgs::msg::Pose target_pose;
        eigen_matrix_to_pose_msg(biTei, target_pose);
        cartesian_client_robot2.goTo(
            fkine_robot2_topic, target_pose,
            rclcpp::Duration::from_seconds(scale_time * action["duration_robot2"].as<double>()), start_time, wait_traj);
      }
      else if (action["offset_robot2_EE"])
      {
        // activate robot pose control
        Eigen::Matrix<double, 4, 4> fkine2_T_fkine2postgrasp;
        fkine2_T_fkine2postgrasp.setIdentity();

        std::vector<double> offset_vec = action["offset_robot2_EE"].as<std::vector<double>>();
        if (offset_vec.size() == 3)
        {
          fkine2_T_fkine2postgrasp.block<3, 1>(0, 3) = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]);
        }
        else
        {
          throw std::runtime_error("offset_robot2_EE must be a vector of size 3");
        }

        geometry_msgs::msg::Pose target_pose;
        eigen_matrix_to_pose_msg(fkine2_T_fkine2postgrasp, target_pose);

        bool wait_traj = true;
        if (action["wait_robot2"])
          wait_traj = action["wait_robot2"].as<bool>();

        cartesian_client_robot2.goTo_EE(
            fkine_robot2_topic, target_pose,
            rclcpp::Duration::from_seconds(scale_time * action["duration_robot2"].as<double>()), rclcpp::Time(0),
            wait_traj);
      }

      if (action["activate_robot1_pose_control"])
        set_activate_status(activate_robot_pose_control_1_client, action["activate_robot1_pose_control"].as<bool>());

      if (action["activate_robot2_pose_control"])
        set_activate_status(activate_robot_pose_control_2_client, action["activate_robot2_pose_control"].as<bool>());

      // GRIPPER: in simulation the grasp is modeled by the robots-object simulator
      // (gripper_robotX / grasp_force_robotX task fields are only used on the real setup).

      if (use_ekf)
      {
        if (action["ekf_object_grasped"])
        {
          set_activate_status(ekf_client_object_grasped, action["ekf_object_grasped"].as<bool>(), true);
          if (simulation)
          {
            set_activate_status(simulator_client_object_grasped, action["ekf_object_grasped"].as<bool>(), true);
          }
        }
      }

      // CONTROLLERS
      if (use_internal_force_control)
      {
        if (action["activate_force_control_robot1"])
          set_activate_status(actvate_internal_force_control_1_client,
                              action["activate_force_control_robot1"].as<bool>());
        if (action["activate_force_control_robot2"])
          set_activate_status(actvate_internal_force_control_2_client,
                              action["activate_force_control_robot2"].as<bool>());
      }
      if (use_object_pose_control)
      {
        if (action["activate_pose_control_robot1"])
          set_activate_status(activate_object_pose_control_1_client, action["activate_pose_control_robot1"].as<bool>());
        if (action["activate_pose_control_robot2"])
          set_activate_status(activate_object_pose_control_2_client, action["activate_pose_control_robot2"].as<bool>());
      }

      // COOPERATIVE MOVEMENT
      if (action["command"])
      {
        if (action["command"].as<std::string>() == "movec_cooperative")
        {
          std::cout << "Cartesian movement"
                    << "\n";

          Eigen::Matrix<double, 4, 4> bTo_final;
          bTo_final.setIdentity();

          if (action["bTo_des"])
            read_transform(action["bTo_des"], bTo_final, construction_config_yaml);
          else if (action["offset_coop_object_pose"])
          {
            auto object_pose_ptr = dual_arm_control::waitForMessage<geometry_msgs::msg::PoseStamped>(obj_pose_topic, node);
            auto object_pose = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(object_pose_ptr);
            Eigen::Matrix4d bTo = Eigen::Matrix<double, 4, 4>::Identity();
            pose_msg_to_eigen_matrix(object_pose->pose, bTo);
            std::cout << "Object pose: \n" << bTo << std::endl;

            bTo_final = bTo;
            if (action["offset_coop_object_pose"])
            {
              std::vector<double> offset_vec = action["offset_coop_object_pose"].as<std::vector<double>>();
              if (offset_vec.size() == 3)
              {
                bTo_final.block<3, 1>(0, 3) =
                    bTo_final.block<3, 1>(0, 3) + Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]);
              }
              else
              {
                throw std::runtime_error("offset_coop_object_pose must be a vector of size 3");
              }
            }
          }
          auto goal_msg = dual_arm_control_interfaces::action::CartesianTrajectory::Goal();
          goal_msg.trajectory.points.resize(1);
          goal_msg.trajectory.points[0].time_from_start =
              rclcpp::Duration::from_seconds(scale_time * action["duration_cooperative"].as<double>());
          eigen_matrix_to_pose_msg(bTo_final, goal_msg.trajectory.points[0].pose);

          std::cout << "bTo_final: \n" << bTo_final << std::endl;
          std::cout << "duration_cooperative: " << action["duration_cooperative"].as<double>() << "\n";

          goal_msg.trajectory.header.stamp = rclcpp::Time(0);
          cartesian_client_object_traj.goTo(obj_pose_topic, goal_msg);
        }
      }

      // cooperative object placement
      if (action["command"])
      {
        if (action["command"].as<std::string>() == "cooperative_object_placement")
        {
          set_activate_status(force_placement_service_client, true);
          wait_for_enter("Wait for placement to finish");
          set_activate_status(force_placement_service_client, false);
        }
      }

      if (action["sleep_time"])
        rclcpp::sleep_for(std::chrono::seconds(static_cast<int>(action["sleep_time"].as<double>())));

      // Robot1 null space activation
      handle_null_space_activation(action, service_client_robot1_null_motion, "q_null_activate_robot1",
                                   "q_null_robot1");

      // Robot2 null space activation
      handle_null_space_activation(action, service_client_robot2_null_motion, "q_null_activate_robot2",
                                   "q_null_robot2");

      // filter aruco
      if (action["robot1_filter_aruco"] && !simulation)
      {
        rcl_interfaces::srv::SetParameters::Request::SharedPtr set_parameter_request =
            std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        std::vector<int64_t> aruco_ids = action["robot1_filter_aruco"].as<std::vector<int64_t>>();
        set_parameter_request->parameters.push_back(
            rclcpp::Parameter("remove_aruco_ids", aruco_ids).to_parameter_msg());
        call_service(aurco_pose_conversion_parameter_client1, set_parameter_request,
                     rcl_interfaces::srv::SetParameters::Response::SharedPtr());
      }
      if (action["robot2_filter_aruco"] && !simulation)
      {
        rcl_interfaces::srv::SetParameters::Request::SharedPtr set_parameter_request =
            std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        std::vector<int64_t> aruco_ids = action["robot2_filter_aruco"].as<std::vector<int64_t>>();
        set_parameter_request->parameters.push_back(
            rclcpp::Parameter("remove_aruco_ids", aruco_ids).to_parameter_msg());
        call_service(aurco_pose_conversion_parameter_client2, set_parameter_request,
                     rcl_interfaces::srv::SetParameters::Response::SharedPtr());
      }
    }
    ekf_filter_initialized = false;  // reset for next task
  }
  rclcpp::shutdown();
  return 0;
}
