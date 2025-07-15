#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PredatorController : public rclcpp::Node {
public:
  PredatorController() : Node("predator_controller") {
    // TODO: Subscribe to predator turtle pose
    sub_ = create_subscription<geometry_msgs::msg::Pose>(
      "/predator/pose", 10,
      std::bind(&PredatorController::pose_callback, this, _1));

    // TODO: Create cmd_vel publisher
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/predator/cmd_vel", 10);
  }

private:
  void pose_callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg) {
    // TODO: Implement hunting algorithm
    // TODO: Check prey proximity (collision detection)
    auto cmd = geometry_msgs::msg::Twist();
    cmd.angular.z = 0.5;  // Example turning motion
    pub_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};
