#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

class PreyController : public rclcpp::Node {
public:
  PreyController() : Node("prey_controller") {
    // TODO: Create synchronized subscribers for all prey turtles
    // Example for 2 prey turtles:
    sub1_.subscribe(this, "/turtle1/pose");
    sub2_.subscribe(this, "/turtle2/pose");
    
    sync_ = std::make_shared<message_filters::TimeSynchronizer<
      geometry_msgs::msg::Pose,
      geometry_msgs::msg::Pose>>(sub1_, sub2_, 10);
    sync_->registerCallback(std::bind(&PreyController::pose_callback, this, _1, _2));

    // TODO: Create cmd_vel publishers for each prey
    pub1_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  }

private:
  void pose_callback(
    const geometry_msgs::msg::Pose::ConstSharedPtr &pose1,
    const geometry_msgs::msg::Pose::ConstSharedPtr &pose2) 
  {
    // TODO: Implement flocking/swarming algorithm
    // TODO: Check for predator proximity (collision detection)
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 1.0;  // Example constant motion
    pub1_->publish(cmd);
  }

  // TODO: Add message_filters subscribers and sync
  message_filters::Subscriber<geometry_msgs::msg::Pose> sub1_, sub2_;
  std::shared_ptr<message_filters::TimeSynchronizer<
    geometry_msgs::msg::Pose,
    geometry_msgs::msg::Pose>> sync_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_;
};
