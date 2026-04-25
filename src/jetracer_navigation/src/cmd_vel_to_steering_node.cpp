#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelToSteeringNode : public rclcpp::Node
{
public:
  CmdVelToSteeringNode()
  : Node("cmd_vel_to_steering")
  {
    // Parameters
    this->declare_parameter<std::string>("input_topic", "cmd_vel_nav");
    this->declare_parameter<std::string>("output_topic", "cmd_vel_nav_steer");
    this->declare_parameter<double>("wheelbase", 0.255);
    this->declare_parameter<double>("max_steering_angle", 0.60);
    this->declare_parameter<double>("min_speed_for_steering", 0.05);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    min_speed_for_steering_ = this->get_parameter("min_speed_for_steering").as_double();

    // Pub/Sub
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, 10, std::bind(&CmdVelToSteeringNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Initialized C++ Ackermann Bridge: %s -> %s", 
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto out_msg = geometry_msgs::msg::Twist();
    
    double velocity = msg->linear.x;
    double yaw_rate = msg->angular.z;

    // Expert Math: Ackermann Inverse Kinematics
    // steering_angle = atan(yaw_rate * wheelbase / velocity)
    
    double steering_angle = 0.0;
    if (std::abs(velocity) > min_speed_for_steering_) {
      steering_angle = std::atan(yaw_rate * wheelbase_ / velocity);
    } else {
      // Avoid singularity at rest: If stationary, steering doesn't map to yaw rate.
      steering_angle = 0.0;
    }

    // Clamp to hardware limits
    steering_angle = std::max(std::min(steering_angle, max_steering_angle_), -max_steering_angle_);

    out_msg.linear.x = velocity;
    out_msg.angular.z = steering_angle;

    pub_->publish(out_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  double wheelbase_;
  double max_steering_angle_;
  double min_speed_for_steering_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToSteeringNode>());
  rclcpp::shutdown();
  return 0;
}
