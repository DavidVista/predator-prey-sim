#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <random>

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        // Create publisher for turtle velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/prey_turtle/cmd_vel", 10);
        
        // Create subscriber for turtle pose
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle/pose", 10,
            std::bind(&TurtleController::pose_callback, this, std::placeholders::_1)
        );
        
        // Initialize random number generator
        random_engine_ = std::mt19937(std::random_device{}());
        linear_dist_ = std::uniform_real_distribution<double>(0.2, 0.6);  // Significantly decreased speed range
        angular_dist_ = std::uniform_real_distribution<double>(-0.5, 0.5);  // Significantly decreased turn rate
        direction_change_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);
        
        // Border limits (turtlesim window is 11x11)
        border_min_ = 0.5;
        border_max_ = 10.5;
        
        // Start movement timer
        movement_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update every 100ms
            std::bind(&TurtleController::update_movement, this)
        );
        
        // Start direction change timer
        direction_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),  // Change direction every 3 seconds
            std::bind(&TurtleController::change_direction, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Turtle Controller initialized");
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }
    
    void update_movement()
    {
        if (!pose_received_) {
            return;  // Wait for first pose message
        }
        
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        // Check if turtle is near borders
        bool near_border = false;
        
        if (current_pose_.x <= border_min_ + 0.5) {
            // Near left border, turn right
            cmd_vel.angular.z = -1.0;
            near_border = true;
        } else if (current_pose_.x >= border_max_ - 0.5) {
            // Near right border, turn left
            cmd_vel.angular.z = 1.0;
            near_border = true;
        } else if (current_pose_.y <= border_min_ + 0.5) {
            // Near bottom border, turn up
            cmd_vel.angular.z = 1.0;
            near_border = true;
        } else if (current_pose_.y >= border_max_ - 0.5) {
            // Near top border, turn down
            cmd_vel.angular.z = -1.0;
            near_border = true;
        }
        
        if (near_border) {
            // When near border, slow down and turn
            cmd_vel.linear.x = 0.5;
        } else {
            // Normal movement
            cmd_vel.linear.x = current_linear_velocity_;
            cmd_vel.angular.z = current_angular_velocity_;
        }
        
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    void change_direction()
    {
        if (!pose_received_) {
            return;
        }
        
        // Randomly change direction
        if (direction_change_dist_(random_engine_) < 0.3) {  // 30% chance to change direction
            current_linear_velocity_ = linear_dist_(random_engine_);
            current_angular_velocity_ = angular_dist_(random_engine_);
            
            RCLCPP_DEBUG(this->get_logger(), "Changed direction: linear=%.2f, angular=%.2f", 
                        current_linear_velocity_, current_angular_velocity_);
        }
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr movement_timer_;
    rclcpp::TimerBase::SharedPtr direction_timer_;
    
    turtlesim::msg::Pose current_pose_;
    bool pose_received_ = false;
    
    // Movement parameters
    double current_linear_velocity_ = 0.4;  // Significantly decreased default speed
    double current_angular_velocity_ = 0.0;
    
    // Border limits
    double border_min_;
    double border_max_;
    
    // Random number generation
    std::mt19937 random_engine_;
    std::uniform_real_distribution<double> linear_dist_;
    std::uniform_real_distribution<double> angular_dist_;
    std::uniform_real_distribution<double> direction_change_dist_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 