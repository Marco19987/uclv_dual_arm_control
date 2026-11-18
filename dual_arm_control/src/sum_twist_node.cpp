#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class SumTwistNode : public rclcpp::Node
{
public:
    SumTwistNode()
    : Node("sum_twist_node")
    {
        sub1_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "twist1", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                last_twist1_ = *msg;
                publish_sum();
            });

        sub2_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "twist2", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                last_twist2_ = *msg;
                publish_sum();
            });

        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_sum", rclcpp::SensorDataQoS());
    }

private:
    void publish_sum()
    {
        geometry_msgs::msg::TwistStamped sum;
        // Use the latest header (you can change this logic as needed)
        sum.header.stamp = this->now();
        sum.header.frame_id = last_twist1_.header.frame_id.empty() ? last_twist2_.header.frame_id : last_twist1_.header.frame_id;

        sum.twist.linear.x  = last_twist1_.twist.linear.x  + last_twist2_.twist.linear.x;
        sum.twist.linear.y  = last_twist1_.twist.linear.y  + last_twist2_.twist.linear.y;
        sum.twist.linear.z  = last_twist1_.twist.linear.z  + last_twist2_.twist.linear.z;
        sum.twist.angular.x = last_twist1_.twist.angular.x + last_twist2_.twist.angular.x;
        sum.twist.angular.y = last_twist1_.twist.angular.y + last_twist2_.twist.angular.y;
        sum.twist.angular.z = last_twist1_.twist.angular.z + last_twist2_.twist.angular.z;
        pub_->publish(sum);
    }

    geometry_msgs::msg::TwistStamped last_twist1_;
    geometry_msgs::msg::TwistStamped last_twist2_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub1_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub2_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumTwistNode>());
    rclcpp::shutdown();
    return 0;
}