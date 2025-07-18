#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <random>
#include <cmath>

class EnergyPredatorController : public rclcpp::Node
{
public:
    EnergyPredatorController() : Node("energy_predator_controller")
    {
        // Create publisher for predator velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/predator_turtle/cmd_vel", 10);
        
        // Create service clients for killing and spawning
        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        
        // Create subscriber for predator pose
        predator_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/predator_turtle/pose", 10,
            std::bind(&EnergyPredatorController::predator_pose_callback, this, std::placeholders::_1)
        );
        
        // Create subscriber for prey pose
        prey_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle/pose", 10,
            std::bind(&EnergyPredatorController::prey_pose_callback, this, std::placeholders::_1)
        );
        
        // Energy system parameters (like a cheetah) - ENHANCED for Boids hunting
        max_energy_ = 150.0;           // More maximum energy (150%)
        current_energy_ = max_energy_; // Start with full energy
        energy_drain_rate_ = 1.0;      // Reduced energy drain (more efficient)
        energy_recovery_rate_ = 3.0;   // Faster energy recovery
        min_energy_to_hunt_ = 15.0;    // Lower energy threshold for hunting
        sprint_energy_cost_ = 3.0;     // Reduced sprint energy cost
        
        // Hunting parameters - Appropriate for turtlesim (11x11 space)
        detection_range_ = 8.0;        // Detection range (most of the map)
        stalking_range_ = 5.0;         // Stalking range (half the map)
        attack_range_ = 2.0;           // Attack range (close proximity)
        catch_range_ = 0.8;            // Catch range (very close)
        
        // Speed parameters (cheetah-like) - Realistic speeds for turtlesim
        stalking_speed_ = 0.4;         // Slow, stealthy stalking
        hunting_speed_ = 1.2;          // Medium hunting speed
        sprint_speed_ = 2.0;           // Fast sprint speed
        
        // Respawn parameters
        respawn_delay_ = 3;            // Seconds to wait before respawning prey
        
        // Border limits
        border_min_ = 0.5;
        border_max_ = 10.5;
        
        // Initialize poses and state
        predator_pose_received_ = false;
        prey_pose_received_ = false;
        prey_caught_ = false;
        catch_cooldown_ = false;
        is_sprinting_ = false;
        last_energy_log_ = 0;
        current_linear_velocity_ = 0.0;
        
        // Initialize random number generator for respawn positions
        random_engine_ = std::mt19937(std::random_device{}());
        respawn_x_dist_ = std::uniform_real_distribution<double>(1.0, 9.0);
        respawn_y_dist_ = std::uniform_real_distribution<double>(1.0, 9.0);
        
        // Start movement timer
        movement_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update every 100ms
            std::bind(&EnergyPredatorController::update_movement, this)
        );
        
        // Start energy management timer
        energy_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update energy every 100ms
            std::bind(&EnergyPredatorController::update_energy, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Energy Predator Controller initialized (Cheetah Mode)");
        RCLCPP_INFO(this->get_logger(), "Max Energy: %.1f, Drain Rate: %.1f/s, Recovery Rate: %.1f/s", 
                   max_energy_, energy_drain_rate_, energy_recovery_rate_);
        RCLCPP_INFO(this->get_logger(), "Detection: %.1f, Stalking: %.1f, Attack: %.1f, Catch: %.1f", 
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
    
    void update_energy()
    {
        // Calculate energy change based on movement
        if (current_linear_velocity_ > 0.1) {
            // Moving - drain energy
            double energy_drain = energy_drain_rate_ * 0.1; // 100ms = 0.1s
            if (is_sprinting_) {
                energy_drain += sprint_energy_cost_ * 0.1; // Extra cost for sprinting
            }
            current_energy_ -= energy_drain;
        } else {
            // Resting - recover energy
            double energy_recovery = energy_recovery_rate_ * 0.1; // 100ms = 0.1s
            current_energy_ += energy_recovery;
        }
        
        // Clamp energy between 0 and max
        current_energy_ = std::max(0.0, std::min(max_energy_, current_energy_));
        
        // Log energy status occasionally
        static int log_counter = 0;
        if (++log_counter % 100 == 0) { // Every 10 seconds
            RCLCPP_INFO(this->get_logger(), "⚡ Energy: %.1f/%.1f (%.1f%%) - Speed: %.2f", 
                       current_energy_, max_energy_, (current_energy_/max_energy_)*100, current_linear_velocity_);
        }
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
        
        // Check if we have enough energy to hunt
        if (current_energy_ < min_energy_to_hunt_) {
            rest_behavior(cmd_vel);
        } else if (distance_to_prey <= detection_range_) {
            // Prey detected - implement energy-based hunting behavior
            energy_based_hunting(cmd_vel, distance_to_prey, dx, dy);
        } else {
            // Prey not detected - patrol behavior
            patrol_behavior(cmd_vel);
        }
        
        // Apply border constraints
        apply_border_constraints(cmd_vel);
        
        // Store linear velocity for energy calculation
        current_linear_velocity_ = cmd_vel.linear.x;
        
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    void energy_based_hunting(geometry_msgs::msg::Twist& cmd_vel, double distance, double dx, double dy)
    {
        // Calculate angle to prey
        double target_angle = std::atan2(dy, dx);
        
        // Calculate angle difference
        double angle_diff = target_angle - predator_pose_.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        // Determine speed based on distance and energy (cheetah-like behavior)
        double linear_speed;
        is_sprinting_ = false;
        
        if (distance <= attack_range_) {
            // Very close - decide whether to sprint based on energy
            if (current_energy_ > 30.0) {  // Lower energy threshold for sprinting
                linear_speed = sprint_speed_;
                is_sprinting_ = true;
                RCLCPP_DEBUG(this->get_logger(), "🏃 SPRINT ATTACK - Distance: %.2f, Energy: %.1f", distance, current_energy_);
            } else {
                linear_speed = hunting_speed_;
                RCLCPP_DEBUG(this->get_logger(), "⚡ NORMAL ATTACK - Distance: %.2f, Energy: %.1f", distance, current_energy_);
            }
        } else if (distance <= stalking_range_) {
            // Within stalking range - hunting speed
            linear_speed = hunting_speed_;
            RCLCPP_DEBUG(this->get_logger(), "🎯 HUNTING MODE - Distance: %.2f, Energy: %.1f", distance, current_energy_);
        } else {
            // Far away - energy-efficient stalking
            linear_speed = stalking_speed_;
            RCLCPP_DEBUG(this->get_logger(), "🦁 STALKING MODE - Distance: %.2f, Energy: %.1f", distance, current_energy_);
        }
        
        // Set velocities
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = 1.5 * angle_diff;  // Proportional control for turning
        
        // Log hunting behavior occasionally
        static int log_counter = 0;
        if (++log_counter % 50 == 0) {  // Log every 5 seconds
            RCLCPP_INFO(this->get_logger(), "🦁 Hunting prey - Distance: %.2f, Speed: %.2f, Energy: %.1f", 
                       distance, linear_speed, current_energy_);
        }
    }
    
    void rest_behavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // No movement - rest to recover energy
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        
        static int rest_log_counter = 0;
        if (++rest_log_counter % 200 == 0) {  // Log every 20 seconds
            RCLCPP_INFO(this->get_logger(), "😴 RESTING - Energy too low (%.1f/%.1f), recovering...", 
                       current_energy_, max_energy_);
        }
    }
    
    void patrol_behavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Energy-efficient patrol behavior when prey is not detected
        // Move slowly toward center of the map
        double dx = 5.5 - predator_pose_.x;
        double dy = 5.5 - predator_pose_.y;
        double target_angle = std::atan2(dy, dx);
        
        double angle_diff = target_angle - predator_pose_.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        cmd_vel.linear.x = 0.2;  // Very slow patrol speed (energy efficient)
        cmd_vel.angular.z = 0.3 * angle_diff;
        
        static int patrol_log_counter = 0;
        if (++patrol_log_counter % 300 == 0) {  // Log every 30 seconds
            RCLCPP_INFO(this->get_logger(), "🚶 Patrolling - No prey detected, Energy: %.1f", current_energy_);
        }
    }
    
    void catch_prey()
    {
        if (catch_cooldown_) {
            return;  // Already processing a catch
        }
        
        catch_cooldown_ = true;
        prey_caught_ = true;
        
        RCLCPP_INFO(this->get_logger(), "🎯 PREY CAUGHT! Distance: %.2f, Energy remaining: %.1f", catch_range_, current_energy_);
        
        // Kill the prey
        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_request->name = "prey_turtle";
        
        auto kill_future = kill_client_->async_send_request(
            kill_request,
            std::bind(&EnergyPredatorController::kill_callback, this, std::placeholders::_1)
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
                std::bind(&EnergyPredatorController::respawn_prey, this)
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
            std::bind(&EnergyPredatorController::respawn_callback, this, std::placeholders::_1)
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
            
            cmd_vel.linear.x = 0.3;  // Slow down near borders
            cmd_vel.angular.z = 2.0 * angle_diff;  // Turn toward center
        }
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr predator_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_;
    rclcpp::TimerBase::SharedPtr movement_timer_;
    rclcpp::TimerBase::SharedPtr energy_timer_;
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
    bool is_sprinting_;
    int last_energy_log_;
    double current_linear_velocity_;
    
    // Energy system
    double max_energy_;
    double current_energy_;
    double energy_drain_rate_;
    double energy_recovery_rate_;
    double min_energy_to_hunt_;
    double sprint_energy_cost_;
    
    // Hunting parameters
    double detection_range_;
    double stalking_range_;
    double attack_range_;
    double catch_range_;
    
    // Speed parameters
    double stalking_speed_;
    double hunting_speed_;
    double sprint_speed_;
    
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
    auto node = std::make_shared<EnergyPredatorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 