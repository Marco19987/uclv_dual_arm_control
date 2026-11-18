#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class ForcePlacementNode : public rclcpp::Node
{
public:
  ForcePlacementNode() : Node("force_placement_node"), running_(false)
  {
    // Subscriber to WrenchStamped
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "input_wrench", 1, std::bind(&ForcePlacementNode::wrench_callback, this, std::placeholders::_1));

    // Publisher for TwistStamped
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("output_twist", 1);

    // Service to set running
    running_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/set_running",
        std::bind(&ForcePlacementNode::set_running_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    if (!running_)
      return;

    double force_norm_ =
        std::sqrt(msg->wrench.force.x * msg->wrench.force.x + msg->wrench.force.y * msg->wrench.force.y +
                  msg->wrench.force.z * msg->wrench.force.z);
    if (initial_force_norm_ < 0)
    {
      initial_force_norm_ = force_norm_;
    }

    if (abs(initial_force_norm_ - force_norm_) >= 1.0)
    {
      RCLCPP_INFO(this->get_logger(), "Force norm stabilized. Stopping.");
      running_ = false;

      publish_zero_twist();
      return;
    }
    // Example: Map force directly to linear velocity, torque to angular velocity
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = msg->header.frame_id;

    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = -0.001;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    twist_pub_->publish(twist_msg);
  }

  void publish_zero_twist()
  {
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";  // Adjust frame_id as needed

    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    for (int i = 0; i < 10; ++i)  // Publish multiple times to ensure reception
    {
      twist_pub_->publish(twist_msg);
      rclcpp::sleep_for(100ms);
    }
  }

  void set_running_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    running_ = request->data;
    response->success = true;
    response->message = running_ ? "Node is running." : "Node is stopped.";
    RCLCPP_INFO(this->get_logger(), "Set running: %s", running_ ? "true" : "false");
    if (!running_)
    {
      initial_force_norm_ = -1.0;  // Reset initial force norm when starting/stopping
      publish_zero_twist();
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr running_srv_;
  bool running_;
  double initial_force_norm_ = -1.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForcePlacementNode>());
  rclcpp::shutdown();
  return 0;
}