/** This node implement the EKF for the robots_object_system system
 * The service takes as input the path to a yaml file containing the parameters of the objects and the topics where to
 * subscribe and read the poses, and the object name to be tracked. When a new service request is received, the node
 * subscribes to the topics and starts the EKF for the requested object.
 *
 *
 *
 *
 * */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "dual_arm_control_interfaces/srv/ekf_service.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "../ekf_ikfom/robot_spring_object_system_ikfom.hpp"
#include "../include/geometry_helper.hpp"

#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "../include/utils_read_yaml.hpp"

#define MAXIMUM_ITER 10 // check the meaning

class EKFServer : public rclcpp::Node
{
public:
  EKFServer() : Node("ekf_server")
  {
    // declare parameters
    this->declare_parameter<double>("sample_time", 0.001);
    this->get_parameter("sample_time", this->sample_time_);

    this->declare_parameter<double>("stop_calibration_param", 0.001);
    this->get_parameter("stop_calibration_param", this->stop_calibration_param_);

    // declare parameters
    this->declare_parameter<double>("occlusion_base", 1.0);
    this->get_parameter("occlusion_base", this->occlusion_base_);

    // declare parameters
    this->declare_parameter<int>("occlusion_exp_limit", 10);
    this->get_parameter("occlusion_exp_limit", this->occlusion_exp_limit_);

    this->declare_parameter<double>("publishing_sample_time", 0.01);
    this->get_parameter("publishing_sample_time", this->publishing_sample_time_);

    this->declare_parameter<std::string>("robot_1_prefix", "robot_1");
    this->get_parameter("robot_1_prefix", this->robot_1_prefix_);

    this->declare_parameter<std::string>("robot_2_prefix", "robot_2");
    this->get_parameter("robot_2_prefix", this->robot_2_prefix_);

    this->declare_parameter<std::string>("base_frame_name", "base_frame");
    this->get_parameter("base_frame_name", this->base_frame_name);

    declare_ros_parameters();

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    // initialize publishers
    object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/object_pose", qos);
    object_twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("ekf/object_twist", qos);
    transform_b2Tb1_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/b2Tb1_filtered", qos);
    transform_b1Tb2_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/b1Tb2_filtered", qos);
    filtered_wrench_1_publisher_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("ekf/wrench_robot1_filtered", qos);
    filtered_wrench_2_publisher_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("ekf/wrench_robot2_filtered", qos);

    fkine_robot1_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/fkine1_filtered", 1);
    fkine_robot2_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/fkine2_filtered", 1);

    oTg1_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/oTg1_filtered", qos);
    oTg2_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/oTg2_filtered", qos);

    // initialize twist subscribers
    int index = 0;
    twist_robot1_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->robot_1_prefix_ + "/twist_fkine", qos,
        [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
          this->twist_fkine_callback(msg, index);
        });
    index++;
    twist_robot2_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->robot_2_prefix_ + "/twist_fkine", qos,
        [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
          this->twist_fkine_callback(msg, index);
        });

    // Create the service server
    server_ = this->create_service<dual_arm_control_interfaces::srv::EKFService>(
        "ekf_service",
        std::bind(&EKFServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "EKFService service server ready!");

    // create service
    server_object_grasped_ = this->create_service<std_srvs::srv::SetBool>(
        "ekf_set_object_grasped",
        std::bind(&EKFServer::handle_grasp_service_request, this, std::placeholders::_1, std::placeholders::_2));

    server_use_measures_ = this->create_service<std_srvs::srv::SetBool>(
        "ekf_set_use_measures",
        std::bind(&EKFServer::handle_use_measure_callback, this, std::placeholders::_1, std::placeholders::_2));

    cb_execution_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("ekf/callback_execution_time", 1);

    external_wrench_estimate_publisher_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("ekf/external_wrench_estimate", 1);

    internal_wrench_estimate_1_publisher =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("ekf/internal_wrench_estimate_1", 1);

    internal_wrench_estimate_2_publisher =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("ekf/internal_wrench_estimate_2", 1);

    e1To_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/e1To_topic", 1);
    e2To_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf/e2To_topic", 1);

    reaction_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("ekf/reaction_wrench", 1);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ekf/object_marker", 1);

    // debug publisher
    debug_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ekf/debug_msg", 1);

    // iniit variables
    y_.setZero();
    init_P_ = W_default_;
  }

private:
  void reset_filter()
  {
    filter_initialized_ = false;
    timer_->cancel();
    timer_publish_->cancel();

    y_.setZero();
    V_.setZero();
    num_frames_ = 0;

    process_params_.W_noise = W_default_;

    // remove the subscribers
    pose_subscribers_.clear();

    // remove the publishers
    filtered_pose_publishers_.clear();

    b1Tb2_convergence_status_ = false;

    // reset grasp detection
    object_grasped_[0] = false;
    object_grasped_[1] = false;

    mass_estimated_flag_ = false;
    mass_estimated_ = 0.0;
    mass_estimated_counter_ = 0;

    selection_matrix_vector_robot1_.clear();
    selection_matrix_vector_robot2_.clear();
  }

  void handle_service_request(const std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Request> request,
                              std::shared_ptr<dual_arm_control_interfaces::srv::EKFService::Response> response)
  {
    if (filter_initialized_)
    {
      RCLCPP_WARN(this->get_logger(), "Filter already initialized : the filter will be reset");
      reset_filter();
    }
    ekf_activation_time = this->now();

    RCLCPP_INFO(this->get_logger(), "Received request");
    // save initial state
    save_initial_state(request);

    // read yaml file
    read_yaml_file(request->yaml_file_path.data.c_str(), request->object_name.data.c_str());
    object_id_++;

    // initialize the EKF
    V_.resize(num_frames_ * 14 + 12 + 14, num_frames_ * 14 + 12 + 14);
    // ! handle V
    // V_.setIdentity();
    // V_ = V_ * 1e10;

    ekf_ikfom_.change_x(state_sim_);
    std::cout << "initial state ekf \n " << state_sim_;
    ekf_ikfom_.change_P(init_P_);
    std::cout << "initial P matrix ekf \n " << init_P_;

    std::fill(epsi, epsi + state_dof, 0.001);
    ekf_ikfom_.init_dyn_runtime_share(get_f_, df_dx_, df_dw_, MAXIMUM_ITER, epsi);

    // start the estimation timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(sample_time_ * 1000)),
                                     std::bind(&EKFServer::ekf_callback, this));

    // timer for publishing the msgs
    timer_publish_ = this->create_wall_timer(std::chrono::milliseconds((int)(publishing_sample_time_ * 1000)),
                                             std::bind(&EKFServer::publish_callback, this));

    // initialize wrench subscribers
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    int index = 0;
    wrench_robot1_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        this->robot_1_prefix_ + "/wrench", qos,
        [this, index](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        { this->wrench_callback(msg, index); });
    index++;
    wrench_robot2_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        this->robot_2_prefix_ + "/wrench", qos,
        [this, index](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        { this->wrench_callback(msg, index); });

    // initialize fkine subscribers
    index = 0;
    fkine_robot1_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->robot_1_prefix_ + "/fkine", qos,
        [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        { this->fkine_callback(msg, index); });
    index++;
    fkine_robot2_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->robot_2_prefix_ + "/fkine", qos,
        [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        { this->fkine_callback(msg, index); });

    // return the response
    filter_initialized_ = true;
    response->success = filter_initialized_;
  }

  void ekf_callback()
  {
    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "Masure pose received vector: " << this->pose_measure_received_.transpose() << "\n"
              << std::endl;
    std::cout << "Masure force received vector: " << this->force_measure_received_.transpose() << "\n";
    std::cout << "Masure fkine received vector: " << this->fkine_measure_received_.transpose() << "\n";

    // update covariance on the base of the received measurements
    // for (int i = 0; i < 2 * num_frames_; i++)
    // {
    //   if (pose_measure_received_(i))
    //   {
    //     V_.block<7, 7>(i * 7, i * 7) = V_single_measure_;
    //     occlusion_counter_(i) = 0;
    //   }
    //   else
    //   {
    //     // V_.block<7, 7>(i * 7, i * 7) = Eigen::Matrix<double, 7, 7>::Identity() * 1e10;

    //     if (occlusion_counter_(i) < occlusion_exp_limit_)
    //     {
    //       occlusion_counter_(i)++;
    //       V_.block<7, 7>(i * 7, i * 7) = std::pow(occlusion_base_, occlusion_counter_(i)) * V_single_measure_;
    //     }
    //   }
    //   pose_measure_received_(i) = false;
    // }
    // for (int i = 0; i < 2; i++)
    // {
    //   if (force_measure_received_(i))
    //   {
    //     V_.block<6, 6>(num_frames_ * 14 + i * 6, num_frames_ * 14 + i * 6) = V_forces_.block<6, 6>(i * 6, i * 6);
    //     occlusion_counter_(i + num_frames_ * 2) = 0;
    //   }
    //   else
    //   {
    //     // V_.block<6, 6>(num_frames_ * 14 + i * 6, num_frames_ * 14 + i * 6) =
    //     //     Eigen::Matrix<double, 6, 6>::Identity() * 1e10;

    //     if (occlusion_counter_(i + num_frames_ * 2) < occlusion_exp_limit_)
    //     {
    //       occlusion_counter_(i + num_frames_ * 2)++;
    //       V_.block<6, 6>(num_frames_ * 14 + i * 6, num_frames_ * 14 + i * 6) =
    //           std::pow(occlusion_base_, occlusion_counter_(i + num_frames_ * 2)) * V_forces_.block<6, 6>(i * 6, i *
    //           6);
    //     }
    //   }
    //   force_measure_received_(i) = false;

    //   if (fkine_measure_received_(i))
    //   {
    //     V_.block<7, 7>(num_frames_ * 14 + 12 + i * 7, num_frames_ * 14 + 12 + i * 7) =
    //         V_fkine_robots_.block<7, 7>(i * 7, i * 7);
    //     occlusion_counter_(i + num_frames_ * 2 + 2) = 0;
    //   }
    //   else
    //   {
    //     // V_.block<7, 7>(num_frames_ * 14 + 12 + i * 7, num_frames_ * 14 + 12 + i * 7) =
    //     //     Eigen::Matrix<double, 7, 7>::Identity() * 1e10;

    //     if (occlusion_counter_(i + num_frames_ * 2 + 2) < occlusion_exp_limit_)
    //     {
    //       occlusion_counter_(i + num_frames_ * 2 + 2)++;
    //       V_.block<7, 7>(num_frames_ * 14 + 12 + i * 7, num_frames_ * 14 + 12 + i * 7) =
    //           std::pow(occlusion_base_, occlusion_counter_(i + num_frames_ * 2 + 2)) * V_fkine_robots_.block<7, 7>(i
    //           * 7, i * 7);
    //     }
    //   }
    //  fkine_measure_received_(i) = false;
    // }
    // std::cout << "\nOcclusion counter: " << occlusion_counter_.transpose() << "\n";

    // ! Fill fields of the measurement
    wrench_measure e1he1_e1_measured;
    wrench_measure e2he2_e2_measured;
    SEN b1_pose_e1_measured;
    SEN b2_pose_e2_measured;
    std::vector<SEN> object_pose_measured;

    int index_force_measure1 = num_frames_ * 14 + 0 * 6;
    int index_force_measure2 = num_frames_ * 14 + 1 * 6;
    int index_fkine_measure1 = num_frames_ * 14 + 12 + 0 * 7;
    int index_fkine_measure2 = num_frames_ * 14 + 12 + 1 * 7;

    e1he1_e1_measured.force << y_.block<3, 1>(index_force_measure1, 0);
    e1he1_e1_measured.torque << y_.block(index_force_measure1 + 3, 0, 3, 1);

    e2he2_e2_measured.force << y_.block(index_force_measure2, 0, 3, 1);
    e2he2_e2_measured.torque << y_.block(index_force_measure2 + 3, 0, 3, 1);

    Eigen::Vector<double, 7> vec_7dim = y_.block(index_fkine_measure1, 0, 7, 1);
    uclv::geometry_helper::pose_to_matrix(vec_7dim, b1_pose_e1_measured.mat);

    vec_7dim = y_.block(index_fkine_measure2, 0, 7, 1);
    uclv::geometry_helper::pose_to_matrix(vec_7dim, b2_pose_e2_measured.mat);

    SEN pose_measure_tmp;
    process_params_.num_pose_measure_robot1 = 0;
    process_params_.num_pose_measure_robot2 = 0;
    for (int i = 0; i < 2 * num_frames_; i++)
    {
      if (pose_measure_received_(i))
      {
        vec_7dim = y_.block(i * 7, 0, 7, 1);
        uclv::geometry_helper::pose_to_matrix(vec_7dim, pose_measure_tmp.mat);
        object_pose_measured.push_back(pose_measure_tmp);

        if (i < num_frames_)
          process_params_.num_pose_measure_robot1++;
        else
          process_params_.num_pose_measure_robot2++;
      }

      pose_measure_received_(i) = false;
    }

    int num_pose_measure = object_pose_measured.size();

    std::cout << "Number of pose measures: " << num_pose_measure << "\n";

    // MTK::MyManifold<double, num_pose_measure> h_(wrench_measure(), wrench_measure(), SEN(), SEN(),
    //                                              std::vector<SEN>(num_pose_measure, SEN()));

    //----------------------------------------

    //* prediction step
    ekf_ikfom_.predict(sample_time_, process_params_.W_noise, u_);

    // std::cout << "PREDICT: \n" << ekf_ikfom_.get_x() << "\n";

    //* correction step
    // use the right manifold measurement function on the base of num_pose_measure
    if (use_measures_)
    {
      switch (num_pose_measure)
      {
      case 0:
      {
        MTK::MyManifold<double, 0> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<0>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 1:
      {
        MTK::MyManifold<double, 1> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<1>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 2:
      {
        MTK::MyManifold<double, 2> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<2>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 3:
      {
        MTK::MyManifold<double, 3> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<3>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 4:
      {
        MTK::MyManifold<double, 4> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<4>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 5:
      {
        MTK::MyManifold<double, 5> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<5>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 6:
      {
        MTK::MyManifold<double, 6> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<6>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 7:
      {
        MTK::MyManifold<double, 7> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<7>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 8:
      {
        MTK::MyManifold<double, 8> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<8>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 9:
      {
        MTK::MyManifold<double, 9> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                      object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<9>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 10:
      {
        MTK::MyManifold<double, 10> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                       object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<10>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 11:
      {
        MTK::MyManifold<double, 11> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                       object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<11>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      case 12:
      {
        MTK::MyManifold<double, 12> h_(e1he1_e1_measured, e2he2_e2_measured, b1_pose_e1_measured, b2_pose_e2_measured,
                                       object_pose_measured);
        auto h_dyn_runtime_share_ =
            std::bind(h_dyn_runtime_share<12>, std::placeholders::_1, std::placeholders::_2, process_params_);
        ekf_ikfom_.update_iterated_dyn_runtime_share(h_, h_dyn_runtime_share_);
        break;
      }
      default:
        RCLCPP_ERROR(this->get_logger(), "Unsupported number of pose measures: %d", num_pose_measure);
        break;
      }
    }
    // std::cout << "CORRECTION STEP: \n" << ekf_ikfom_.get_x() << "\n";

    // check b1Tb2 convergence
    if (b1Tb2_convergence_status_ == false && last_pose_msg_robot1_ && last_pose_msg_robot2_)
    {
      rclcpp::Time time_measure_1 = last_pose_msg_robot1_->header.stamp;
      rclcpp::Time time_measure_2 = last_pose_msg_robot2_->header.stamp;

      if (this->now() - ekf_activation_time > rclcpp::Duration::from_seconds(5.0))
      {
        if (abs((time_measure_1 - time_measure_2).seconds()) < 0.15 &&
            this->now() - time_measure_1 < rclcpp::Duration::from_seconds(0.15) &&
            this->now() - time_measure_2 < rclcpp::Duration::from_seconds(0.15))
        {
          Eigen::Matrix<double, 7, 1> aruco_pose_1;
          aruco_pose_1 << last_pose_msg_robot1_->pose.position.x, last_pose_msg_robot1_->pose.position.y,
              last_pose_msg_robot1_->pose.position.z, last_pose_msg_robot1_->pose.orientation.w,
              last_pose_msg_robot1_->pose.orientation.x, last_pose_msg_robot1_->pose.orientation.y,
              last_pose_msg_robot1_->pose.orientation.z;

          Eigen::Matrix<double, 7, 1> aruco_pose_2;
          aruco_pose_2 << last_pose_msg_robot2_->pose.position.x, last_pose_msg_robot2_->pose.position.y,
              last_pose_msg_robot2_->pose.position.z, last_pose_msg_robot2_->pose.orientation.w,
              last_pose_msg_robot2_->pose.orientation.x, last_pose_msg_robot2_->pose.orientation.y,
              last_pose_msg_robot2_->pose.orientation.z;

          check_b1Tb2_convergence(aruco_pose_1, aruco_pose_2);
        }
      }
      else
      {
        std::cout << "Waiting calibration time " << (this->now() - ekf_activation_time).seconds() << "\n";
      }
    }
    std::cout << "b1Tb2_convergence_status_: " << b1Tb2_convergence_status_ << "\n";

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

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() << " s\n";

    std_msgs::msg::Float64 msg;
    msg.data = elapsed.count();
    cb_execution_time_pub_->publish(msg);
  }

  void fill_pose_message_from_T(const Eigen::Matrix4d &T, geometry_msgs::msg::PoseStamped &pose_msg,
                                rclcpp::Time timestamp = rclcpp::Time(0))
  {
    pose_msg.header.stamp = timestamp;
    pose_msg.pose.position.x = T(0, 3);
    pose_msg.pose.position.y = T(1, 3);
    pose_msg.pose.position.z = T(2, 3);
    // Initialize orientation from rotation matrix
    Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
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
  void publish_callback()
  {
    // get the state estimate
    state_sim_ = ekf_ikfom_.get_x();

    // check if nan

    if (!state_sim_.bTo.mat.allFinite())
    {
      std::cerr << "ERROR: NaN detected in state vector x_old!" << std::endl;
      std::abort();
    } 

    // get the filtered measures
    const int num_pose_measure = 2 * 6; // number of pose measurements
    process_params_.num_pose_measure_robot1 = num_frames_;
    process_params_.num_pose_measure_robot2 = num_frames_;
    MTK::MyManifold<double, num_pose_measure> h_(wrench_measure(), wrench_measure(), SEN(), SEN(),
                                                 std::vector<SEN>(num_pose_measure, SEN()));
    h_ = get_h<num_pose_measure>(state_sim_, process_params_);

    // get time stamp for the messages
    auto timestamp = this->now();

    // publish messages
    geometry_msgs::msg::PoseStamped pose_msg;
    for (int i = 0; i < process_params_.num_pose_measure_robot1; i++)
    {
      fill_pose_message_from_T(h_.object_pose[i].mat, pose_msg, timestamp);
      filtered_pose_publishers_[i]->publish(pose_msg);
    }
    for (int i = process_params_.num_pose_measure_robot1;
         i < process_params_.num_pose_measure_robot1 + process_params_.num_pose_measure_robot2; i++)
    {
      fill_pose_message_from_T(h_.object_pose[i].mat, pose_msg, timestamp);
      filtered_pose_publishers_[i]->publish(pose_msg);
    }

    // publish wrench
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = timestamp;
    wrench_msg.wrench.force.x = h_.e1he1_e1.force(0);
    wrench_msg.wrench.force.y = h_.e1he1_e1.force(1);
    wrench_msg.wrench.force.z = h_.e1he1_e1.force(2);
    wrench_msg.wrench.torque.x = h_.e1he1_e1.torque(0);
    wrench_msg.wrench.torque.y = h_.e1he1_e1.torque(1);
    wrench_msg.wrench.torque.z = h_.e1he1_e1.torque(2);
    filtered_wrench_1_publisher_->publish(wrench_msg);

    wrench_msg.wrench.force.x = h_.e2he2_e2.force(0);
    wrench_msg.wrench.force.y = h_.e2he2_e2.force(1);
    wrench_msg.wrench.force.z = h_.e2he2_e2.force(2);
    wrench_msg.wrench.torque.x = h_.e2he2_e2.torque(0);
    wrench_msg.wrench.torque.y = h_.e2he2_e2.torque(1);
    wrench_msg.wrench.torque.z = h_.e2he2_e2.torque(2);
    filtered_wrench_2_publisher_->publish(wrench_msg);

    // publish pose
    pose_msg.header.frame_id = "world";
    fill_pose_message_from_T(state_sim_.bTo.mat, pose_msg, timestamp);
    object_pose_publisher_->publish(pose_msg);

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
    robot1_fkine_msg.header.frame_id = "world"; // this->robot_1_prefix_;
    fill_pose_message_from_T(state_sim_.b1Te1.mat, robot1_fkine_msg, timestamp);
    fkine_robot1_pub_->publish(robot1_fkine_msg);

    geometry_msgs::msg::PoseStamped robot2_fkine_msg;
    robot2_fkine_msg.header.frame_id = "base_link"; // this->robot_2_prefix_;
    fill_pose_message_from_T(state_sim_.b2Te2.mat, robot2_fkine_msg, timestamp);
    fkine_robot2_pub_->publish(robot2_fkine_msg);

    // publish the transform b2Tb1
    geometry_msgs::msg::PoseStamped b2Tb1_msg;
    fill_pose_message_from_T(state_sim_.b2Tb1.mat, b2Tb1_msg, timestamp);
    transform_b2Tb1_publisher_->publish(b2Tb1_msg);

    // publish oTg1
    geometry_msgs::msg::PoseStamped oTg1_msg;
    oTg1_msg.header.frame_id = "object_pose";
    fill_pose_message_from_T(state_sim_.oTg1.mat, oTg1_msg, timestamp);

    oTg1_pub_->publish(oTg1_msg);

    // publish oTg2
    geometry_msgs::msg::PoseStamped oTg2_msg;
    oTg2_msg.header.frame_id = "object_pose";
    fill_pose_message_from_T(state_sim_.oTg2.mat, oTg2_msg, timestamp);
    oTg2_pub_->publish(oTg2_msg);

    {
      // publish the transform b1Tb2
      Eigen::Matrix<double, 4, 4> b1Tb2_filtered;
      geometry_msgs::msg::PoseStamped transform_b1Tb2_msg;
      transform_b1Tb2_msg.header.frame_id = this->measure_1_base_frame;
      b1Tb2_filtered = state_sim_.b2Tb1.mat.inverse();
      fill_pose_message_from_T(b1Tb2_filtered, transform_b1Tb2_msg, timestamp);
      transform_b1Tb2_publisher_->publish(transform_b1Tb2_msg);
    }

    // {
    //   // publish debug message
    //   std_msgs::msg::Float64MultiArray debug_msg;
    //   debug_msg.data.resize(state_dof + y_.size() + u_.size());
    //   for (int i = 0; i < state_dof; i++)
    //   {
    //     debug_msg.data[i] = x_hat_k_k(i);
    //   }
    //   for (int i = 0; i < y_.size(); i++)
    //   {
    //     debug_msg.data[state_dof + i] = y_(i);
    //   }
    //   for (int i = 0; i < u_.size(); i++)
    //   {
    //     debug_msg.data[state_dof + y_.size() + i] = u_(i);
    //   }
    //   debug_publisher_->publish(debug_msg);
    // }

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
    external_wrench_estimate_publisher_->publish(external_wrench_msg);

    // Publish internal wrench for robot 1
    geometry_msgs::msg::WrenchStamped internal_wrench_1_msg;
    internal_wrench_1_msg.header.stamp = timestamp;
    internal_wrench_1_msg.wrench.force.x = internal_wrench(0);
    internal_wrench_1_msg.wrench.force.y = internal_wrench(1);
    internal_wrench_1_msg.wrench.force.z = internal_wrench(2);
    internal_wrench_1_msg.wrench.torque.x = internal_wrench(3);
    internal_wrench_1_msg.wrench.torque.y = internal_wrench(4);
    internal_wrench_1_msg.wrench.torque.z = internal_wrench(5);
    internal_wrench_estimate_1_publisher->publish(internal_wrench_1_msg);

    // Publish internal wrench for robot 2
    geometry_msgs::msg::WrenchStamped internal_wrench_2_msg;
    internal_wrench_2_msg.header.stamp = timestamp;
    internal_wrench_2_msg.wrench.force.x = internal_wrench(6);
    internal_wrench_2_msg.wrench.force.y = internal_wrench(7);
    internal_wrench_2_msg.wrench.force.z = internal_wrench(8);
    internal_wrench_2_msg.wrench.torque.x = internal_wrench(9);
    internal_wrench_2_msg.wrench.torque.y = internal_wrench(10);
    internal_wrench_2_msg.wrench.torque.z = internal_wrench(11);
    internal_wrench_estimate_2_publisher->publish(internal_wrench_2_msg);

    // publish e1To and e2To
    Eigen::Matrix<double, 4, 4> e1To =
        state_sim_.b1Te1.mat.inverse() * process_params_.bTb1.mat.inverse() * state_sim_.bTo.mat;
    Eigen::Matrix<double, 4, 4> e2To =
        state_sim_.b2Te2.mat.inverse() * state_sim_.b2Tb1.mat * process_params_.bTb1.mat.inverse() * state_sim_.bTo.mat;

    // Publish e1To
    if (publish_eeTo_)
    {
      geometry_msgs::msg::PoseStamped e1To_msg;
      e1To_msg.header.frame_id = "iiwa_end_effector";
      fill_pose_message_from_T(e1To, e1To_msg, timestamp);
      e1To_pub_->publish(e1To_msg);

      // Publish e2To
      geometry_msgs::msg::PoseStamped e2To_msg;
      e2To_msg.header.frame_id = "yaskawa_end_effector";
      fill_pose_message_from_T(e2To, e2To_msg, timestamp);
      e2To_pub_->publish(e2To_msg);
    }

    // Publish reaction wrench
    // geometry_msgs::msg::WrenchStamped reaction_wrench_msg;
    // reaction_wrench_msg.header.stamp = timestamp;
    // reaction_wrench_msg.wrench.force.x = state_sim_.f_v(0);
    // reaction_wrench_msg.wrench.force.y = state_sim_.f_v(1);
    // reaction_wrench_msg.wrench.force.z = state_sim_.f_v(2);
    // reaction_wrench_msg.wrench.torque.x = state_sim_.tau_v(0);
    // reaction_wrench_msg.wrench.torque.y = state_sim_.tau_v(1);
    // reaction_wrench_msg.wrench.torque.z = state_sim_.tau_v(2);
    // reaction_wrench_pub_->publish(reaction_wrench_msg);

    // marker pub
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = timestamp;

    marker.ns = "object_shape";
    marker.id = object_id_;

    marker.type = visualization_msgs::msg::Marker::CUBE;

    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = state_sim_.bTo.mat(0, 3);
    marker.pose.position.y = state_sim_.bTo.mat(1, 3);
    marker.pose.position.z = state_sim_.bTo.mat(2, 3);
    Eigen::Quaterniond q = Eigen::Quaterniond(state_sim_.bTo.mat.block<3, 3>(0, 0));
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();

    marker.scale.x = object_size_(0);
    marker.scale.y = object_size_(1);
    marker.scale.z = object_size_(2);

    marker.color.r = object_color_(0);
    marker.color.g = object_color_(1);
    marker.color.b = object_color_(2);
    marker.color.a = 1.0; // Don't forget to set the alpha!

    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker_pub_->publish(marker);
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
    Eigen::Matrix<double, 4, 4> b1Tb2;
    b1Tb2.setIdentity();
    read_transform(config["b1Tb2"], b1Tb2);
    std::cout << "b1Tb2: \n"
              << b1Tb2 << std::endl;
    Eigen::Matrix<double, 4, 4> b2Tb1 = b1Tb2.inverse();
    state_sim_.b2Tb1.mat = b2Tb1;

    // read bTb1
    Eigen::Matrix<double, 4, 4> bTb1;
    bTb1.setIdentity();
    read_transform(config["bTb1"], bTb1);
    std::cout << "bTb1: \n"
              << bTb1 << std::endl;
    process_params_.bTb1.mat = bTb1;

    YAML::Node object_node = config[object_name];

    // read viscous friction
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

    // update reaction wrench (the object lays on the plane)
    // state_sim_.f_v =
    // -process_params_.M.block<3, 3>(0, 0) * state_sim_.bTo.mat.block<3, 3>(0, 0).transpose() * process_params_.bg;

    // read object_size
    object_size_.setZero();
    std::vector<double> object_size = object_node["object_size"].as<std::vector<double>>();
    object_size_ << object_size[0], object_size[1], object_size[2];
    std::cout << "Object Size: \n"
              << object_size_ << std::endl;

    // read object color
    object_color_.setZero();
    std::vector<double> object_color = object_node["rgb_color"].as<std::vector<double>>();
    object_color_ << object_color[0], object_color[1], object_color[2];
    std::cout << "rgb_color: \n"
              << object_color_ << std::endl;

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

    // std::cout << "H_sel matrix:\n" << process_params_.H_sel << std::endl;

    // // compute selection matrix negate
    // fill_selection_matrix_negate(H_sel_1_diag, Htmp1);
    // fill_selection_matrix_negate(H_sel_2_diag, Htmp2);

    // process_params_.H_sel_neg.resize(Htmp1.rows() + Htmp2.rows(), Htmp1.cols() + Htmp2.cols());
    // process_params_.H_sel_neg.setZero();
    // process_params_.H_sel_neg.block(0, 0, Htmp1.rows(), Htmp1.cols()) = Htmp1;
    // process_params_.H_sel_neg.block(Htmp1.rows(), Htmp1.cols(), Htmp2.rows(), Htmp2.cols()) = Htmp2;

    // std::cout << "\n H_sel matrix negate: \n" << process_params_.H_sel_neg << std::endl;

    // compute oTg1 and oTg2 from the state_sim_ variables

    state_sim_.oTg1.mat = state_sim_.bTo.mat.inverse() * process_params_.bTb1.mat * state_sim_.b1Te1.mat;
    std::cout << "oTg1: \n"
              << state_sim_.oTg1.mat << std::endl;

    state_sim_.oTg2.mat =
        state_sim_.bTo.mat.inverse() * process_params_.bTb1.mat * state_sim_.b2Tb1.mat.inverse() * state_sim_.b2Te2.mat;
    std::cout << "oTg2: \n"
              << state_sim_.oTg2.mat << std::endl;

    process_params_.K1 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.K2 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.B2 = Eigen::Matrix<double, 6, 6>::Zero();
    process_params_.B2 = Eigen::Matrix<double, 6, 6>::Zero();

    printProcessParams(process_params_);

    // read names of frames published
    std::vector<std::string> frame_names;
    for (const auto &transformation : object_node["aruco_transforms"])
    {
      frame_names.push_back(transformation["name"].as<std::string>());
      RCLCPP_INFO(this->get_logger(), "Frame: %s", transformation["name"].as<std::string>().c_str());
    }

    // instantiate the subscribers to the pose topics
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    int num_frames = frame_names.size();
    int index = 0;
    for (const auto &frame_name : frame_names)
    {
      pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
          this->robot_1_prefix_ + "/" + object_name + "/" + frame_name + "/pose", qos,
          [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          { this->pose_callback(msg, index); }));

      pose_subscribers_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
          this->robot_2_prefix_ + "/" + object_name + "/" + frame_name + "/pose", qos,
          [this, num_frames, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            this->pose_callback(msg, num_frames + index);
          }));

      index++;
    }
    num_frames_ = num_frames;

    // resize the output variable
    y_.resize(num_frames_ * 14 + 12 + 14, 1);
    y_.setZero();

    y_filtered_.resize(num_frames_ * 14 + 12 + 14, 1);
    y_filtered_.setZero();

    occlusion_counter_.resize(num_frames_ * 2 + 2 + 2, 1);
    occlusion_counter_.setOnes();
    occlusion_counter_ = occlusion_counter_ * occlusion_exp_limit_;

    pose_measure_received_.resize(num_frames_ * 2);
    pose_measure_received_.setZero();
    force_measure_received_.setZero();
    fkine_measure_received_.setZero();

    // initialize filtered pose publishers
    for (int i = 0; i < num_frames_; i++)
    {
      filtered_pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "ekf" + this->robot_1_prefix_ + "/" + object_name + "/" + frame_names[i] + "/pose_filtered", 1));
    }
    for (int i = num_frames; i < 2 * num_frames_; i++)
    {
      filtered_pose_publishers_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "ekf" + this->robot_2_prefix_ + "/" + object_name + "/" + frame_names[i - num_frames] + "/pose_filtered",
          1));
    }
  }

  void fkine_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const int &index)
  {
    // RCLCPP_INFO(this->get_logger(), "Received fkine from %d", index);
    // if (this->object_grasped_[index])
    // {
    int position_measure_vector = num_frames_ * 2 * 7 + 12 + 7 * index;
    // y_.block(position_measure_vector, 0, 7, 1) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
    //     msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    Eigen::Matrix<double, 4, 1> q;
    q << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    // ensure quaternion scalar part being positive
    if (q.w() < 0)
    {
      q = -q;
    }

    y_.block<4, 1>(position_measure_vector + 3, 0) = q;
    y_.block<3, 1>(position_measure_vector, 0) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    fkine_measure_received_(index) = true;

    // }
  }

  void twist_fkine_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg, const int &index)
  {
    if (index == 0)
    {
      u_.e1ve1(0) = msg->twist.linear.x;
      u_.e1ve1(1) = msg->twist.linear.y;
      u_.e1ve1(2) = msg->twist.linear.z;
      u_.e1omegae1(0) = msg->twist.angular.x;
      u_.e1omegae1(1) = msg->twist.angular.y;
      u_.e1omegae1(2) = msg->twist.angular.z;
    }

    if (index == 1)
    {
      u_.e2ve2(0) = msg->twist.linear.x;
      u_.e2ve2(1) = msg->twist.linear.y;
      u_.e2ve2(2) = msg->twist.linear.z;
      u_.e2omegae2(0) = msg->twist.angular.x;
      u_.e2omegae2(1) = msg->twist.angular.y;
      u_.e2omegae2(2) = msg->twist.angular.z;
    }

    if (index < 1)
    {
      last_twist_msg_robot1_ = msg;
    }
    else
    {
      last_twist_msg_robot2_ = msg;
    }
  }

  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, const int &index)
  {
    // RCLCPP_INFO(this->get_logger(), "Received wrench from %d", index);
    // if (mass_estimated_flag_ == false)
    // {
    //   this->measured_wrench_robots_mass_estimation_.block<6, 1>(index * 6, 0) << msg->wrench.force.x,
    //       msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
    //       0.0;  //!  msg->wrench.torque.z = 0 due pivoting link
    //   return;
    // }

    if (this->object_grasped_[index])
    {
      int position_measure_vector = num_frames_ * 2 * 7 + 6 * index;
      y_.block(position_measure_vector, 0, 6, 1) << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
          msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z; //= 0 due pivoting link

      force_measure_received_(index) = true;
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const int &index)
  {
    // RCLCPP_INFO(this->get_logger(), "Received pose from %d", index);
    // this->y_.block<7, 1>(index * 7, 0) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
    //     msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    if (measure_1_base_frame == "" && index < num_frames_)
    {
      measure_1_base_frame = msg->header.frame_id;
    }
    if (measure_2_base_frame == "" && index >= num_frames_)
    {
      measure_2_base_frame = msg->header.frame_id;
    }

    // ensure quaternion continuity
    Eigen::Matrix<double, 4, 1> q;
    q << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
    // ensure quaternion scalar part being positive
    if (q.w() < 0)
    {
      q = -q;
    }

    y_.block<4, 1>(index * 7 + 3, 0) = q;
    y_.block<3, 1>(index * 7, 0) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    pose_measure_received_(index) = true;

    // store last available msg
    if (index < num_frames_)
    {
      last_pose_msg_robot1_ = msg;
    }
    else
    {
      last_pose_msg_robot2_ = msg;
    }
  }

  void check_b1Tb2_convergence(Eigen::Matrix<double, 7, 1> measure_robot1, Eigen::Matrix<double, 7, 1> measure_robot2)
  {
    state_sim_ = ekf_ikfom_.get_x();

    Eigen::Matrix<double, 4, 4> b2To_measure2;
    b2To_measure2.setIdentity();
    uclv::geometry_helper::pose_to_matrix<double>(measure_robot2, b2To_measure2);

    Eigen::Matrix<double, 4, 4> bTo_frommeasure2 =
        process_params_.bTb1.mat * state_sim_.b2Tb1.mat.inverse() * b2To_measure2;

    Eigen::Matrix<double, 4, 4> b1To_measure1;
    b1To_measure1.setIdentity();
    uclv::geometry_helper::pose_to_matrix<double>(measure_robot1, b1To_measure1);
    Eigen::Matrix<double, 4, 4> bTo_frommeasure1 = process_params_.bTb1.mat * b1To_measure1;

    // evaluate error
    Eigen::Matrix<double, 4, 4> error = bTo_frommeasure1.inverse() * bTo_frommeasure2;

    Eigen::Matrix<double, 6, 1> tau_error = SEN::log(error);

    // std::cout << "Error matrix: \n" << error << std::endl;
    // std::cout << "bTo_frommeasure1 matrix: \n" << bTo_frommeasure1 << std::endl;
    // std::cout << "bTo_frommeasure2 matrix: \n" << bTo_frommeasure2 << std::endl;

    // Compute distance of error matrix from identity
    // std::cout << "Error postion norm: " << error.block<3, 1>(0, 3).norm() << std::endl;
    std::cout << "Error tau :" << tau_error << "\n";
    std::cout << "Error tau norm :" << tau_error.norm() << "\n";

    // compute angle axis of the error matrix
    // Eigen::AngleAxisd error_angle_axis(error.block<3, 3>(0, 0));
    // std::cout << "Error angle axis: " << error_angle_axis.angle() << std::endl;
    // std::cout << "Error axis: " << error_angle_axis.axis().transpose() << std::endl;

    std::cout << "Error tau postion norm: " << tau_error.block<3, 1>(0, 0).norm() << std::endl;
    std::cout << "Error tau angle norm: " << tau_error.block<3, 1>(3, 0).norm() << std::endl;

    // if (error.block<3, 1>(0, 3).norm() < 0.007 && error_angle_axis.angle() < 0.01)
    if (tau_error.block<3, 1>(0, 0).norm() < stop_calibration_param_ &&
        tau_error.block<3, 1>(3, 0).norm() < stop_calibration_param_ * 10)
    // if (tau_error.norm() < stop_calibration_param_)

    {
      std::cout << "b1Tb2 converged" << std::endl;
      process_params_.W_noise.block<6, 6>(24, 24) = W_default_.block<6, 6>(24, 24) * 0.0;
      b1Tb2_convergence_status_ = true;
      ekf_ikfom_.change_P(process_params_.W_noise);
    }
    else
    {
      // W_.block<7, 7>(13, 13) = W_default_.block<7, 7>(13, 13);
    }
  }

  void handle_grasp_service_request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (filter_initialized_ == false)
    {
      response->success = false;
      response->message = "EKF not initialized";
      return;
    }

    if (request->data) // object grasped
    {
      object_grasped_[0] = true;
      object_grasped_[1] = true;

      std::cout << "Object grasped" << std::endl;

      // Set oTg frames
      state_sim_ = ekf_ikfom_.get_x();
      state_sim_.oTg1.mat = state_sim_.bTo.mat.inverse() * process_params_.bTb1.mat * state_sim_.b1Te1.mat;
      std::cout << "oTg1: \n"
                << state_sim_.oTg1.mat << std::endl;

      state_sim_.oTg2.mat = state_sim_.bTo.mat.inverse() * process_params_.bTb1.mat * state_sim_.b2Tb1.mat.inverse() *
                            state_sim_.b2Te2.mat;
      std::cout << "oTg2: \n"
                << state_sim_.oTg2.mat << std::endl;

      ekf_ikfom_.change_x(state_sim_);

      std::cout << "EKF state updated: \n";

      // enable contact in the model
      process_params_.K1 = K_1_matrix_;
      process_params_.B1 = B_1_matrix_;
      process_params_.K2 = K_2_matrix_;
      process_params_.B2 = B_2_matrix_;

      // enable graity
      process_params_.bg = bg;

      // if (this->mass_estimated_flag_ == false)
      // {
      //   timer_mass_estimation_ = this->create_wall_timer(std::chrono::milliseconds(10),
      //                                                    std::bind(&EKFServer::mass_estimation_callback, this));
      // }
    }
    else
    {
      object_grasped_[0] = false;
      object_grasped_[1] = false;
      process_params_.K1 = Eigen::Matrix<double, 6, 6>::Zero();
      process_params_.B1 = Eigen::Matrix<double, 6, 6>::Zero();
      process_params_.K2 = Eigen::Matrix<double, 6, 6>::Zero();
      process_params_.B2 = Eigen::Matrix<double, 6, 6>::Zero();

      // state_sim_.f_v.setZero();
      // state_sim_.tau_v.setZero();

      ekf_ikfom_.change_x(state_sim_);

      process_params_.bg = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);

      std::cout << "Object released" << std::endl;
    }

    response->success = true;
    response->message = request->data ? "EKF object grasped True" : "EKF object grasped False";
  }

  void handle_use_measure_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (filter_initialized_ == false)
    {
      response->success = false;
      response->message = "EKF not initialized";
      return;
    }

    use_measures_ = request->data;

    response->success = true;
    response->message = request->data ? "EKF object grasped True" : "EKF object grasped False";
  }

  void mass_estimation_callback()
  {
    // if (mass_estimated_counter_ == 0)
    //   external_wrenches_.resize(6, mass_estimation_samples_);

    // std::cout << "MASS ESIMATION CB " << mass_estimated_counter_ << std::endl;
    // Eigen::Matrix<double, 6, 1> external_wrench;
    // robots_object_system_ptr_->get_object_wrench(measured_wrench_robots_mass_estimation_, external_wrench);

    // external_wrenches_.block<6, 1>(0, mass_estimated_counter_) << external_wrench;

    // mass_estimated_counter_++;
    // if (mass_estimated_counter_ == mass_estimation_samples_)
    // {
    //   mass_estimated_flag_ = true;
    //   double norm_force = 0.0;
    //   Eigen::Vector3d medium_force;
    //   medium_force.setZero();
    //   for (int i = 0; i < mass_estimation_samples_; i++)
    //   {
    //     Eigen::Vector3d force = external_wrenches_.block<3, 1>(0, i);
    //     norm_force = norm_force + force.norm();
    //     medium_force = medium_force + force;
    //   }
    //   medium_force = medium_force / mass_estimation_samples_;
    //   mass_estimated_ = (norm_force / mass_estimation_samples_) / 9.81;

    //   RCLCPP_INFO(this->get_logger(), "Mass estimated: %f", mass_estimated_);
    //   timer_mass_estimation_->cancel();

    //   // set the mass in the system
    //   robots_object_system_ptr_->set_mass(mass_estimated_);
    //   // robots_object_system_ptr_->set_gravity(Eigen::Matrix<double, 3, 1>(0.0, 0.0, -9.81));

    //   // estimate bias external wrench
    //   Eigen::Matrix<double, 6, 1> bias_external_wrench;
    //   bias_external_wrench.setZero();
    //   for (int i = 0; i < mass_estimation_samples_; i++)
    //   {
    //     bias_external_wrench = bias_external_wrench + external_wrenches_.block<6, 1>(0, i); //-
    //     // Eigen::Matrix<double, 6, 1>(medium_force(0), medium_force(1), medium_force(2), 0.0, 0.0, 0.0);
    //   }
    //   bias_external_wrench = bias_external_wrench / mass_estimation_samples_;
    //   // robots_object_system_ptr_->set_ho_bias(bias_external_wrench);
    //   std::cout << "medium_force: \n"
    //             << medium_force << std::endl;
    //   std::cout << "Bias external wrench: \n"
    //             << bias_external_wrench << std::endl;

    //   // enable contact in the model
    //   robots_object_system_ptr_->set_K_1(K_1_matrix_);
    //   robots_object_system_ptr_->set_B_1(B_1_matrix_);
    //   robots_object_system_ptr_->set_K_2(K_2_matrix_);
    //   robots_object_system_ptr_->set_B_2(B_2_matrix_);
    // }
  }

  void declare_ros_parameters()
  {
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    cb_handles_.clear();

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "covariance_state_diagonal";
      desc.description = "covariance_state_diagonal. Provide 12 numbers.";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(
          desc.name,
          std::vector<double>{1e-12, 1e-12, 1e-5, 1e-5, 1e-12, 1e-12, 1e-12, 1e-12, 1e-8, 1e-10, 1e-8, 1e-8}, desc);
      cb_handles_.insert(
          {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                {
             std::vector<double> covariance_state_diagonal = p.as_double_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
             // check if the covariance_state_diagonal has the correct size
             if (covariance_state_diagonal.size() != 12)
             {
               RCLCPP_ERROR(this->get_logger(),
                            "The covariance_state_diagonal parameter must have 10 elements representing the covariance "
                            "on the "
                            "state. The first is the element for the position terms x,y,z. The next, is the covariance "
                            "for "
                            "the quaternion terms qw,qx,qy,qz. The third is the covariance for the linear velocity "
                            "terms "
                            "vx,vy,vz. "
                            "The fourth is the covariance for the angular velocity terms omegax,omegay,omegaz. The "
                            "fifth is the "
                            "covariance of the position part of the calibration matrix between the two robots. "
                            "The sixth is the covariance of the position of the robot 1 fkine as well as the eight"
                            "The seventh is the covariance of the position of the robot 2 fkine as well as the ninth"
                            "The last is the covariance of the orientation part of the calibration matrix between the "
                            "two "
                            "robots. the last two are the covariance of the force and torque measures"
                            "the last two are cov of pos and quat of oTg1 and oTg2");
               return;
             }
             // initialize covariance matrices W and V
             W_default_ << Eigen::Matrix<double, state_dof, state_dof>::Identity() * 1;
             W_default_.block<3, 3>(0, 0) = W_default_.block<3, 3>(0, 0) * covariance_state_diagonal[0];  // bpo
             W_default_.block<3, 3>(3, 3) = W_default_.block<3, 3>(3, 3) * covariance_state_diagonal[1];  // bQo
             W_default_.block<3, 3>(6, 6) = W_default_.block<3, 3>(6, 6) * covariance_state_diagonal[2];  // ovo
             W_default_.block<3, 3>(9, 9) = W_default_.block<3, 3>(9, 9) * covariance_state_diagonal[3];  // oomegao
             W_default_.block<3, 3>(12, 12) =
                 W_default_.block<3, 3>(12, 12) * covariance_state_diagonal[4];  // b1pe1_b1
             W_default_.block<3, 3>(15, 15) = W_default_.block<3, 3>(15, 15) * covariance_state_diagonal[5];  // b1Qe1
             W_default_.block<3, 3>(18, 18) =
                 W_default_.block<3, 3>(18, 18) * covariance_state_diagonal[6];  // b2pe2_b2
             W_default_.block<3, 3>(21, 21) = W_default_.block<3, 3>(21, 21) * covariance_state_diagonal[7];   // b2Qe2
             W_default_.block<3, 3>(24, 24) = W_default_.block<3, 3>(24, 24) * covariance_state_diagonal[8];   // b2pb1
             W_default_.block<3, 3>(27, 27) = W_default_.block<3, 3>(27, 27) * covariance_state_diagonal[9];   // b2Qb1
             W_default_.block<3, 3>(30, 30) = W_default_.block<3, 3>(30, 30) * covariance_state_diagonal[10];  // opg1
             W_default_.block<3, 3>(33, 33) = W_default_.block<3, 3>(33, 33) * covariance_state_diagonal[11];  // oQg1
             W_default_.block<3, 3>(36, 36) = W_default_.block<3, 3>(36, 36) * covariance_state_diagonal[10];  // opg2
             W_default_.block<3, 3>(39, 39) = W_default_.block<3, 3>(39, 39) * covariance_state_diagonal[11];  // oQg2
             //  W_default_.block<3, 3>(42, 42) = W_default_.block<3, 3>(42, 42) * covariance_state_diagonal[12];  //
             //  reaction wrench W_default_.block<3, 3>(45, 45) = W_default_.block<3, 3>(45, 45) *
             //  covariance_state_diagonal[13];  //

             // multiply W_default by the sample time
             W_default_ = W_default_ * sample_time_ / (0.001);  // covariance tuned with sample time == 1e-3
             process_params_.W_noise = W_default_;

             if (filter_initialized_)
             {
               ekf_ikfom_.change_P(process_params_.W_noise);
             } })});
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "covariance_measure_diagonal";
      desc.description = "covariance_measure_diagonal. Provide 16 numbers.";
      desc.additional_constraints = "";
      desc.read_only = false;
      this->declare_parameter(desc.name,
                              std::vector<double>{1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2,
                                                  1e-2, 1e-9, 1e-9, 1e-9, 1e-9},
                              desc);
      cb_handles_.insert(
          {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                {
             std::vector<double> covariance_measure_diagonal = p.as_double_array();
             RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
             // check if the covariance_measure_diagonal has the correct size
             if (covariance_measure_diagonal.size() != 16)
             {
               RCLCPP_ERROR(this->get_logger(),
                            "The covariance_measure_diagonal parameter must have 15 elements representing the "
                            "covariance on the "
                            "measure, the first 3 are the position terms x,y,z, the next 4 are the quaternion terms "
                            "qw,qx,qy,qz.The next 6 are the covariance of the force and torque measures . The last 4 "
                            "are the "
                            "covariance measure of the robot 1 and robot 2 fkine respectively");
               return;
             }

             V_single_measure_ << Eigen::Matrix<double, 6, 6>::Identity() * 1;
             for (int i = 0; i < 6; i++)
             {
               V_single_measure_(i, i) = covariance_measure_diagonal[i];
             }

             std::cout << "\nInitial single measure V covariance matrix\n" << V_single_measure_ << std::endl;

             V_forces_ << Eigen::Matrix<double, 12, 12>::Identity() * 1;

             V_forces_.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
             for (int i = 0; i < 6; i++)
             {
               V_forces_(i, i) = covariance_measure_diagonal[6 + i];
             }

             V_forces_.block<6, 6>(6, 6) = V_forces_.block<6, 6>(0, 0);

             std::cout << "\nInitial forces V covariance matrix\n" << V_forces_ << std::endl;

             V_fkine_robots_ << Eigen::Matrix<double, 12, 12>::Identity() * 1;
             V_fkine_robots_.block<3, 3>(0, 0) = V_fkine_robots_.block<3, 3>(0, 0) * covariance_measure_diagonal[12];
             V_fkine_robots_.block<3, 3>(3, 3) = V_fkine_robots_.block<3, 3>(3, 3) * covariance_measure_diagonal[13];
             V_fkine_robots_.block<3, 3>(6, 6) = V_fkine_robots_.block<3, 3>(6, 6) * covariance_measure_diagonal[14];
             V_fkine_robots_.block<3, 3>(9, 9) = V_fkine_robots_.block<3, 3>(9, 9) * covariance_measure_diagonal[15];

             process_params_.V_pose_measure_rb1 = V_single_measure_;
             process_params_.V_pose_measure_rb2 = V_single_measure_;
             process_params_.V_fkine_measure_rb1 = V_fkine_robots_.block<6, 6>(0, 0);
             process_params_.V_fkine_measure_rb2 = V_fkine_robots_.block<6, 6>(6, 6);
             process_params_.V_wrench_measure_rb1 = V_forces_.block<6, 6>(0, 0);
             process_params_.V_wrench_measure_rb2 = V_forces_.block<6, 6>(6, 6);

             std::cout << "\nInitial fkine V covariance matrix\n" << V_fkine_robots_ << std::endl; })});
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "publish_eeTo";
      desc.description = "Enable publishing of eeTo topics.";
      desc.read_only = false;
      this->declare_parameter(desc.name, false, desc);
      cb_handles_.insert(
          {desc.name, param_subscriber_->add_parameter_callback(desc.name, [this](const rclcpp::Parameter &p)
                                                                {
                                                                  publish_eeTo_ = p.as_bool();
                                                                  RCLCPP_INFO_STREAM(this->get_logger(), "Received an update to parameter " << p);
                                                                  // You can use publish_eeTo to enable/disable publishing of eeTo topics in your code.
                                                                })});
    }

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
             update_selection_matrix(); })});
    }
  }

  void update_selection_matrix()
  {
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
    }
  }

  rclcpp::Service<dual_arm_control_interfaces::srv::EKFService>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_object_grasped_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_use_measures_;

  // subscribers to poseStamped
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscribers_;

  // subscribers to WrenchStamped
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_robot2_sub_;

  // subscribers to fkine
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot2_sub_;

  // twist subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_robot1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_robot2_sub_;

  // timer for ekf update
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_publish_;
  rclcpp::TimerBase::SharedPtr timer_mass_estimation_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr object_twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transform_b2Tb1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transform_b1Tb2_publisher_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> filtered_pose_publishers_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_2_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fkine_robot2_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr oTg1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr oTg2_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cb_execution_time_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr external_wrench_estimate_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr internal_wrench_estimate_1_publisher;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr internal_wrench_estimate_2_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr e1To_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr e2To_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr reaction_wrench_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // strings to attach at the topic name to subscribe
  std::string robot_1_prefix_;
  std::string robot_2_prefix_;

  state_ikfom state_sim_; // state vector (struct)
  input_ikfom u_;         // input vector (struct)
  ProcessParams process_params_;

  Eigen::Matrix<double, Eigen::Dynamic, 1> y_;              // variable to store the measures for the filter [aruco poses,
                                                            // whrences, fkines]
  Eigen::Matrix<double, Eigen::Dynamic, 1> y_filtered_;     // variable to store the measures filtered
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V_; // measurement noise covariance matrix
  Eigen::Matrix<double, state_dof, state_dof> W_default_;   // default covariance matrix for the state
  Eigen::Matrix<double, 6, 6> V_single_measure_;            // covariance matrix for the single object pose measure
  Eigen::Matrix<double, 12, 12> V_forces_;                  // covariance matrix for the forces and torques
  Eigen::Matrix<double, 12, 12> V_fkine_robots_;            // covariance matrix for the fkine measures

  bool filter_initialized_ = false; // flag to check if the filter has been initialized

  double sample_time_;            // sample time for the filter
  double publishing_sample_time_; // sample time for the publishing
  int num_frames_;                // number of frames measuring the object

  // variables for occlusion handling
  Eigen::Vector<bool, Eigen::Dynamic> pose_measure_received_; // this vector indicates if a measure has been received
  Eigen::Vector<bool, 2> force_measure_received_;             // this vector indicates if a force measure has been received
  Eigen::Vector<bool, 2> fkine_measure_received_;             // this vector indicates if a fkine measure has been received

  std::string base_frame_name; // base frame name
  std::string measure_1_base_frame;
  std::string measure_2_base_frame;

  bool b1Tb2_convergence_status_ = false;

  // mass estimation variables
  bool object_grasped_[2] = {false, false}; // use force measures only if the object is grasped
  bool mass_estimated_flag_ = false;
  double mass_estimated_ = 0.0;
  int mass_estimated_counter_ = 0.0;
  int mass_estimation_samples_ = 100;
  Eigen::Matrix<double, 12, 1> measured_wrench_robots_mass_estimation_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> external_wrenches_;

  // last available aruco msg
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_robot1_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_robot2_;

  geometry_msgs::msg::TwistStamped::SharedPtr last_twist_msg_robot1_;
  geometry_msgs::msg::TwistStamped::SharedPtr last_twist_msg_robot2_;

  // store spring parameters
  Eigen::Matrix<double, 6, 6> K_1_matrix_;
  Eigen::Matrix<double, 6, 6> B_1_matrix_;
  Eigen::Matrix<double, 6, 6> B_2_matrix_;
  Eigen::Matrix<double, 6, 6> K_2_matrix_;
  Eigen::Matrix<double, 3, 1> bg;

  // occlusion handling
  Eigen::Vector<int, Eigen::Dynamic> occlusion_counter_;
  int occlusion_exp_limit_ = 25;
  double occlusion_base_ = 1.584893192;

  esekfom::esekf<state_ikfom, process_noise_dof, input_ikfom> ekf_ikfom_;
  esekfom::esekf<state_ikfom, process_noise_dof, input_ikfom>::cov init_P_;
  // bind functions to adapt interface of esekfom
  std::function<Eigen::Matrix<double, state_dof, 1>(state_ikfom &, const input_ikfom &)> get_f_ =
      std::bind(get_f, std::placeholders::_1, std::placeholders::_2, std::ref(process_params_));
  std::function<Eigen::Matrix<double, state_dof, state_dof>(state_ikfom &, const input_ikfom &)> df_dx_ =
      std::bind(df_dx, std::placeholders::_1, std::placeholders::_2, std::ref(process_params_));
  std::function<Eigen::Matrix<double, state_dof, process_noise_dof>(state_ikfom &, const input_ikfom &)> df_dw_ =
      std::bind(df_dw, std::placeholders::_1, std::placeholders::_2, std::ref(process_params_));

  double stop_calibration_param_ = 0.00001;

  double epsi[state_dof] = {0.001};

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> cb_handles_;

  bool publish_eeTo_ = false;
  bool use_measures_ = true;

  Eigen::Matrix<double, 3, 1> object_size_;
  Eigen::Matrix<double, 3, 1> object_color_;
  int32_t object_id_ = 0;

  std::vector<bool> selection_matrix_vector_robot1_;
  std::vector<bool> selection_matrix_vector_robot2_;

  rclcpp::Time ekf_activation_time;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFServer>());
  rclcpp::shutdown();
  return 0;
}