#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <random>
#include <cmath>

class PredatorController : public rclcpp::Node
{
public:
    PredatorController() : Node("predator_controller")
    {
        // Create publisher for predator velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/predator_turtle/cmd_vel", 10);
        
        // Create service clients for killing and spawning
        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        
        // Create subscriber for predator pose
        predator_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/predator_turtle/pose", 10,
            std::bind(&PredatorController::predator_pose_callback, this, std::placeholders::_1)
        );
        
        // Create subscriber for prey pose
        prey_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle/pose", 10,
            std::bind(&PredatorController::prey_pose_callback, this, std::placeholders::_1)
        );
        
        // Hunting parameters - Appropriate for turtlesim (11x11 space)
        detection_range_ = 8.0;  // Detection range (most of the map)
        stalking_range_ = 5.0;   // Stalking range (half the map)
        attack_range_ = 2.0;     // Attack range (close proximity)
        catch_range_ = 0.8;      // Catch range (very close)
        
        // Speed parameters - Realistic cheetah speeds for turtlesim
        stalking_speed_ = 0.4;   // Slow, stealthy stalking
        hunting_speed_ = 1.2;    // Medium hunting speed
        attack_speed_ = 2.0;     // Fast sprint attack
        
        // Respawn parameters
        respawn_delay_ = 3;      // Seconds to wait before respawning prey
        
        // Border limits
        border_min_ = 0.5;
        border_max_ = 10.5;
        
        // Initialize poses and state
        predator_pose_received_ = false;
        prey_pose_received_ = false;
        prey_caught_ = false;
        catch_cooldown_ = false;
        
        // Initialize random number generator for respawn positions
        random_engine_ = std::mt19937(std::random_device{}());
        respawn_x_dist_ = std::uniform_real_distribution<double>(1.0, 9.0);
        respawn_y_dist_ = std::uniform_real_distribution<double>(1.0, 9.0);
        
        // Start movement timer
        movement_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update every 100ms
            std::bind(&PredatorController::update_movement, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Predator Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Detection range: %.1f, Stalking range: %.1f, Attack range: %.1f, Catch range: %.1f", 
                   detection_range_, stalking_range_, attack_range_, catch_range_);
    }

private:
    void predator_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        predator_pose_ = *msg;
        predator_pose_received_ = true;
    }
    
    void prey_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        prey_pose_ = *msg;
        prey_pose_received_ = true;
    }
    
    void update_movement()
    {
        if (!predator_pose_received_ || !prey_pose_received_) {
            return;  // Wait for both poses
        }
        
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        // Calculate distance to prey
        double dx = prey_pose_.x - predator_pose_.x;
        double dy = prey_pose_.y - predator_pose_.y;
        double distance_to_prey = std::sqrt(dx*dx + dy*dy);
        
        // Check if predator caught the prey
        if (distance_to_prey <= catch_range_ && !catch_cooldown_) {
            catch_prey();
            return;
        }
        
        // Check if prey is within detection range
        if (distance_to_prey <= detection_range_) {
            // Prey detected - implement stalking behavior
            stalking_behavior(cmd_vel, distance_to_prey, dx, dy);
        } else {
            // Prey not detected - patrol behavior
            patrol_behavior(cmd_vel);
        }
        
        // Apply border constraints
        apply_border_constraints(cmd_vel);
        
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    void catch_prey()
    {
        if (catch_cooldown_) {
            return;  // Already processing a catch
        }
        
        catch_cooldown_ = true;
        prey_caught_ = true;
        
        RCLCPP_INFO(this->get_logger(), "🎯 PREY CAUGHT! Distance: %.2f", catch_range_);
        
        // Kill the prey
        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_request->name = "prey_turtle";
        
        auto kill_future = kill_client_->async_send_request(
            kill_request,
            std::bind(&PredatorController::kill_callback, this, std::placeholders::_1)
        );
    }
    
    void kill_callback(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Prey turtle killed successfully");
            
            // Start timer to respawn prey
            respawn_timer_ = this->create_wall_timer(
                std::chrono::seconds(respawn_delay_),
                std::bind(&PredatorController::respawn_prey, this)
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to kill prey: %s", e.what());
            catch_cooldown_ = false;  // Reset cooldown on failure
        }
    }
    
    void respawn_prey()
    {
        // Only run once
        respawn_timer_->cancel();
        
        // Generate random spawn position
        double spawn_x = respawn_x_dist_(random_engine_);
        double spawn_y = respawn_y_dist_(random_engine_);
        
        auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawn_request->x = spawn_x;
        spawn_request->y = spawn_y;
        spawn_request->theta = 0.0;
        spawn_request->name = "prey_turtle";
        
        RCLCPP_INFO(this->get_logger(), "🔄 Respawning prey at (%.1f, %.1f)...", spawn_x, spawn_y);
        
        auto spawn_future = spawn_client_->async_send_request(
            spawn_request,
            std::bind(&PredatorController::respawn_callback, this, std::placeholders::_1)
        );
    }
    
    void respawn_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "✅ Prey respawned successfully!");
            
            // Reset state
            prey_caught_ = false;
            catch_cooldown_ = false;
            prey_pose_received_ = false;  // Reset to wait for new prey pose
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to respawn prey: %s", e.what());
            catch_cooldown_ = false;  // Reset cooldown on failure
        }
    }
    
    void stalking_behavior(geometry_msgs::msg::Twist& cmd_vel, double distance, double dx, double dy)
    {
        // Calculate angle to prey
        double target_angle = std::atan2(dy, dx);
        
        // Calculate angle difference
        double angle_diff = target_angle - predator_pose_.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        // Determine speed based on distance (stalking behavior)
        double linear_speed;
        if (distance <= attack_range_) {
            // Very close - attack speed
            linear_speed = attack_speed_;
            RCLCPP_DEBUG(this->get_logger(), "ATTACK MODE - Distance: %.2f", distance);
        } else if (distance <= stalking_range_) {
            // Within stalking range - hunting speed
            linear_speed = hunting_speed_;
            RCLCPP_DEBUG(this->get_logger(), "HUNTING MODE - Distance: %.2f", distance);
        } else {
            // Far away - stalking speed
            linear_speed = stalking_speed_;
            RCLCPP_DEBUG(this->get_logger(), "STALKING MODE - Distance: %.2f", distance);
        }
        
        // Set velocities
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = 1.5 * angle_diff;  // Proportional control for turning
        
        // Log hunting behavior occasionally
        static int log_counter = 0;
        if (++log_counter % 50 == 0) {  // Log every 5 seconds
            RCLCPP_INFO(this->get_logger(), "Hunting prey - Distance: %.2f, Speed: %.2f", distance, linear_speed);
        }
    }
    
    void patrol_behavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Simple patrol behavior when prey is not detected
        // Move toward center of the map
        double dx = 5.5 - predator_pose_.x;
        double dy = 5.5 - predator_pose_.y;
        double target_angle = std::atan2(dy, dx);
        
        double angle_diff = target_angle - predator_pose_.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        cmd_vel.linear.x = 0.3;  // Slow patrol speed
        cmd_vel.angular.z = 0.5 * angle_diff;
        
        static int patrol_log_counter = 0;
        if (++patrol_log_counter % 100 == 0) {  // Log every 10 seconds
            RCLCPP_INFO(this->get_logger(), "Patrolling - No prey detected");
        }
    }
    
    void apply_border_constraints(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Check if predator is near borders
        if (predator_pose_.x <= border_min_ + 0.5 || 
            predator_pose_.x >= border_max_ - 0.5 ||
            predator_pose_.y <= border_min_ + 0.5 || 
            predator_pose_.y >= border_max_ - 0.5) {
            
            // Calculate angle to center to avoid border
            double dx = 5.5 - predator_pose_.x;
            double dy = 5.5 - predator_pose_.y;
            double target_angle = std::atan2(dy, dx);
            
            double angle_diff = target_angle - predator_pose_.theta;
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
            
            cmd_vel.linear.x = 0.5;  // Slow down near borders
            cmd_vel.angular.z = 2.0 * angle_diff;  // Turn toward center
        }
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr predator_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_;
    rclcpp::TimerBase::SharedPtr movement_timer_;
    rclcpp::TimerBase::SharedPtr respawn_timer_;
    
    // Service clients
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    
    turtlesim::msg::Pose predator_pose_;
    turtlesim::msg::Pose prey_pose_;
    bool predator_pose_received_;
    bool prey_pose_received_;
    bool prey_caught_;
    bool catch_cooldown_;
    
    // Hunting parameters
    double detection_range_;
    double stalking_range_;
    double attack_range_;
    double catch_range_;
    
    // Speed parameters
    double stalking_speed_;
    double hunting_speed_;
    double attack_speed_;
    
    // Respawn parameters
    int respawn_delay_;
    
    // Border limits
    double border_min_;
    double border_max_;
    
    // Random number generation for respawn
    std::mt19937 random_engine_;
    std::uniform_real_distribution<double> respawn_x_dist_;
    std::uniform_real_distribution<double> respawn_y_dist_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PredatorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 