#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/msg/color.hpp>
#include <turtlesim/srv/kill.hpp>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <limits>

struct PreyTarget {
    std::string name;
    turtlesim::msg::Pose pose;
    double distance;
    double energy_efficiency;
    double catch_probability;
    bool is_isolated;
    double threat_score;
    bool is_alive;
    
    PreyTarget() : distance(0.0), energy_efficiency(0.0), catch_probability(0.0), 
                   is_isolated(false), threat_score(0.0), is_alive(true) {}
};

class EnergyEfficientPredatorController : public rclcpp::Node
{
public:
    EnergyEfficientPredatorController() : Node("energy_efficient_predator_controller")
    {
        // Create publisher for predator velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/predator_turtle/cmd_vel", 10);
        
        // Create publisher for predator color (pen)
        pen_pub_ = this->create_publisher<turtlesim::msg::Color>("/predator_turtle/set_pen", 10);
        
        // Create service client for killing prey
        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        
        // Create subscribers for all poses
        predator_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/predator_turtle/pose", 10,
            std::bind(&EnergyEfficientPredatorController::predator_pose_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to all prey poses
        prey_pose_sub_1_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle_1/pose", 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                prey_targets_["prey_turtle_1"].pose = *msg;
                prey_targets_["prey_turtle_1"].is_alive = true;
            }
        );
        
        prey_pose_sub_2_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle_2/pose", 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                prey_targets_["prey_turtle_2"].pose = *msg;
                prey_targets_["prey_turtle_2"].is_alive = true;
            }
        );
        
        prey_pose_sub_3_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle_3/pose", 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                prey_targets_["prey_turtle_3"].pose = *msg;
                prey_targets_["prey_turtle_3"].is_alive = true;
            }
        );
        
        // Energy system parameters (INCREASED for higher speeds)
        max_energy_ = 200.0;           // More energy for aggressive hunting (was 200.0)
        current_energy_ = max_energy_; // Start with full energy
        energy_drain_rate_ = 1.2;      // Increased energy drain for higher speeds (was 0.8)
        energy_recovery_rate_ = 5.0;   // Faster energy recovery (was 4.0)
        min_energy_to_hunt_ = 30.0;    // Higher energy threshold for hunting (was 20.0)
        sprint_energy_cost_ = 4.0;     // Higher sprint energy cost (was 2.5)
        
        // Rest system parameters
        is_resting_ = false;
        rest_energy_threshold_ = 25.0;     // Energy level to start resting
        rest_recovery_threshold_ = 70.0;   // Energy level to stop resting
        rest_duration_ = 0.0;
        rest_start_time_ = 0.0;
        rest_position_set_ = false;
        
        // Hunting parameters (INCREASED for more aggressive hunting)
        detection_range_ = 9.0;        // Larger detection range (was 8.0)
        stalking_range_ = 6.0;         // Larger stalking range (was 5.0)
        attack_range_ = 3.0;           // Larger attack range (was 2.0)
        catch_range_ = 1.0;            // Larger catch range (was 0.8)
        isolation_threshold_ = 5.0;    // Larger isolation threshold (was 4.0)
        
        // Speed parameters (INCREASED for more aggressive hunting)
        stalking_speed_ = 1.2;         // Faster stalking (was 0.5)
        hunting_speed_ = 1.5;          // Fast hunting speed (was 1.0)
        sprint_speed_ = 2.5;           // Very fast sprint speed (was 1.8)
        
        // Target selection weights
        distance_weight_ = 0.3;        // Weight for distance factor
        efficiency_weight_ = 0.3;      // Weight for energy efficiency
        isolation_weight_ = 0.2;       // Weight for isolation factor
        probability_weight_ = 0.2;     // Weight for catch probability
        
        // Border limits
        border_min_ = 0.5;
        border_max_ = 10.5;
        
        // Initialize state
        predator_pose_received_ = false;
        current_target_ = nullptr;
        is_sprinting_ = false;
        current_linear_velocity_ = 0.0;
        target_switch_counter_ = 0;
        
        // Initialize prey targets
        initialize_prey_targets();
        
        // Start movement timer
        movement_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update every 100ms
            std::bind(&EnergyEfficientPredatorController::update_movement, this)
        );
        
        // Start energy management timer
        energy_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update energy every 100ms
            std::bind(&EnergyEfficientPredatorController::update_energy, this)
        );
        
        // Start kill completion check timer
        kill_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Check kills every 100ms
            std::bind(&EnergyEfficientPredatorController::check_kill_completion, this)
        );
        
        // Set predator color to lime red
        set_predator_color();
        
        RCLCPP_INFO(this->get_logger(), "Energy Efficient Predator Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Max Energy: %.1f, Drain Rate: %.1f/s, Recovery Rate: %.1f/s", 
                   max_energy_, energy_drain_rate_, energy_recovery_rate_);
        RCLCPP_INFO(this->get_logger(), "Rest System - Start Threshold: %.1f, Recovery Threshold: %.1f", 
                   rest_energy_threshold_, rest_recovery_threshold_);
        RCLCPP_INFO(this->get_logger(), "Target Selection Weights - Distance: %.1f, Efficiency: %.1f, Isolation: %.1f, Probability: %.1f", 
                   distance_weight_, efficiency_weight_, isolation_weight_, probability_weight_);
    }

private:
    void initialize_prey_targets()
    {
        std::vector<std::string> prey_names = {"prey_turtle_1", "prey_turtle_2", "prey_turtle_3"};
        for (const auto& name : prey_names) {
            prey_targets_[name] = PreyTarget();
            prey_targets_[name].name = name;
        }
    }
    
    void set_predator_color()
    {
        auto color_msg = turtlesim::msg::Color();
        // Lime red color: bright red with some green
        color_msg.r = 255;  // Full red
        color_msg.g = 50;   // Some green for lime effect
        color_msg.b = 0;    // No blue
        
        pen_pub_->publish(color_msg);
        RCLCPP_INFO(this->get_logger(), "Predator color set to lime red (R:%d, G:%d, B:%d)", 
                   color_msg.r, color_msg.g, color_msg.b);
    }
    
    void predator_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        predator_pose_ = *msg;
        predator_pose_received_ = true;
    }
    

    
    void update_energy()
    {
        // Calculate energy change based on movement and rest state
        if (is_resting_) {
            // Enhanced energy recovery while resting
            double rest_multiplier = 1.5; // 50% faster recovery when resting
            double energy_recovery = energy_recovery_rate_ * rest_multiplier * 0.1; // 100ms = 0.1s
            current_energy_ += energy_recovery;
            
            // Update rest duration
            rest_duration_ += 0.1; // 100ms
            
            // Check if we should stop resting
            if (current_energy_ >= rest_recovery_threshold_) {
                is_resting_ = false;
                rest_position_set_ = false;
                RCLCPP_INFO(this->get_logger(), "Rest complete! Energy recovered to %.1f/%.1f (%.1f%%) - Duration: %.1fs", 
                           current_energy_, max_energy_, (current_energy_/max_energy_)*100, rest_duration_);
            }
        } else if (current_linear_velocity_ > 0.1) {
            // Moving - drain energy
            double energy_drain = energy_drain_rate_ * 0.1; // 100ms = 0.1s
            if (is_sprinting_) {
                energy_drain += sprint_energy_cost_ * 0.1; // Extra cost for sprinting
            }
            current_energy_ -= energy_drain;
            
            // Check if we should start resting
            if (current_energy_ <= rest_energy_threshold_) {
                is_resting_ = true;
                rest_start_time_ = this->now().seconds();
                rest_duration_ = 0.0;
                rest_position_ = predator_pose_;
                rest_position_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Energy low (%.1f/%.1f) - Starting rest mode at position (%.1f, %.1f)", 
                           current_energy_, max_energy_, rest_position_.x, rest_position_.y);
            }
        } else {
            // Not moving but not in rest mode - normal energy recovery
            double energy_recovery = energy_recovery_rate_ * 0.1; // 100ms = 0.1s
            current_energy_ += energy_recovery;
        }
        
        // Clamp energy between 0 and max
        current_energy_ = std::max(0.0, std::min(max_energy_, current_energy_));
        
        // Log energy status occasionally
        static int log_counter = 0;
        if (++log_counter % 100 == 0) { // Every 10 seconds
<<<<<<< HEAD
            if (is_resting_) {
                RCLCPP_INFO(this->get_logger(), "RESTING - Energy: %.1f/%.1f (%.1f%%) - Rest duration: %.1fs", 
                           current_energy_, max_energy_, (current_energy_/max_energy_)*100, rest_duration_);
            } else {
                RCLCPP_INFO(this->get_logger(), "Energy: %.1f/%.1f (%.1f%%) - Speed: %.2f", 
                           current_energy_, max_energy_, (current_energy_/max_energy_)*100, current_linear_velocity_);
            }
=======
            RCLCPP_INFO(this->get_logger(), "Energy: %.1f/%.1f (%.1f%%) - Speed: %.2f", 
                       current_energy_, max_energy_, (current_energy_/max_energy_)*100, current_linear_velocity_);
>>>>>>> dd0c26786ce801157d5cad41355f269985dc39bb
        }
    }
    
    void update_movement()
    {
        if (!predator_pose_received_) {
            return;  // Wait for predator pose
        }
        
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        // Priority 1: Rest if energy is critically low
        if (is_resting_) {
            rest_behavior(cmd_vel);
        } else {
            // Update all prey distances and scores
            update_prey_targets();
            
            // Select best target
            PreyTarget* best_target = select_best_target();
            
            if (best_target == nullptr) {
                // No valid targets - patrol behavior
                patrol_behavior(cmd_vel);
            } else {
                // Check if we have enough energy to hunt
                if (current_energy_ < min_energy_to_hunt_) {
                    // Force rest mode if energy is too low
                    is_resting_ = true;
                    rest_start_time_ = this->now().seconds();
                    rest_duration_ = 0.0;
                    rest_position_ = predator_pose_;
                    rest_position_set_ = true;
                    RCLCPP_INFO(this->get_logger(), "Energy too low for hunting (%.1f/%.1f) - Forcing rest mode", 
                               current_energy_, max_energy_);
                    rest_behavior(cmd_vel);
                } else {
                    // Hunt the selected target
                    hunt_target(cmd_vel, *best_target);
                }
            }
        }
        
        // Apply border constraints (but allow rest near borders)
        if (!is_resting_) {
            apply_border_constraints(cmd_vel);
        }
        
        // Store linear velocity for energy calculation
        current_linear_velocity_ = cmd_vel.linear.x;
        
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    void update_prey_targets()
    {
        for (auto& pair : prey_targets_) {
            PreyTarget& target = pair.second;
            
            if (!target.is_alive) {
                continue; // Skip dead prey
            }
            
            // Calculate distance to predator
            double dx = target.pose.x - predator_pose_.x;
            double dy = target.pose.y - predator_pose_.y;
            target.distance = std::sqrt(dx*dx + dy*dy);
            
            // Calculate energy efficiency (reward/cost)
            double energy_cost = target.distance * energy_drain_rate_;
            target.energy_efficiency = (energy_cost > 0) ? 1.0 / energy_cost : 1.0;
            
            // Calculate catch probability
            target.catch_probability = calculate_catch_probability(target);
            
            // Check if prey is isolated
            target.is_isolated = check_isolation(target);
            
            // Calculate threat score (combination of factors)
            target.threat_score = calculate_threat_score(target);
        }
    }
    
    PreyTarget* select_best_target()
    {
        PreyTarget* best_target = nullptr;
        double best_score = -1.0;
        
        for (auto& pair : prey_targets_) {
            PreyTarget& target = pair.second;
            
            if (!target.is_alive) {
                continue; // Skip dead prey
            }
            
            if (target.distance > detection_range_) {
                continue; // Skip prey outside detection range
            }
            
            double score = calculate_target_score(target);
            
            if (score > best_score) {
                best_score = score;
                best_target = &target;
            }
        }
        
        // Check if we should switch targets
        if (current_target_ != best_target && best_target != nullptr) {
            target_switch_counter_++;
            if (target_switch_counter_ % 50 == 0) { // Re-evaluate every 5 seconds
                if (current_target_ == nullptr || 
                    calculate_target_score(*best_target) > calculate_target_score(*current_target_) * 1.1) {
                    RCLCPP_INFO(this->get_logger(), "Switching target from %s to %s (Score: %.3f)", 
                               (current_target_ ? current_target_->name.c_str() : "none"),
                               best_target->name.c_str(), best_score);
                    current_target_ = best_target;
                }
            }
        } else if (current_target_ == nullptr && best_target != nullptr) {
            current_target_ = best_target;
            RCLCPP_INFO(this->get_logger(), "Selected initial target: %s (Score: %.3f)", 
                       best_target->name.c_str(), best_score);
        }
        
        return current_target_;
    }
    
    double calculate_target_score(const PreyTarget& target)
    {
        // Normalize factors to 0-1 range
        double distance_factor = 1.0 / (1.0 + target.distance);
        double efficiency_factor = std::min(target.energy_efficiency / 10.0, 1.0); // Normalize efficiency
        double isolation_factor = target.is_isolated ? 1.0 : 0.0;
        double probability_factor = target.catch_probability;
        
        // Weighted combination
        double score = distance_factor * distance_weight_ +
                      efficiency_factor * efficiency_weight_ +
                      isolation_factor * isolation_weight_ +
                      probability_factor * probability_weight_;
        
        return score;
    }
    
    double calculate_catch_probability(const PreyTarget& target)
    {
        // Base probability based on distance
        double base_prob = 1.0 / (1.0 + target.distance);
        
        // Energy factor (more energy = higher probability)
        double energy_factor = current_energy_ / max_energy_;
        
        // Isolation factor (isolated prey = easier to catch)
        double isolation_factor = target.is_isolated ? 1.2 : 1.0;
        
        return std::min(base_prob * energy_factor * isolation_factor, 1.0);
    }
    
    bool check_isolation(const PreyTarget& target)
    {
        double min_distance_to_others = std::numeric_limits<double>::max();
        
        for (const auto& pair : prey_targets_) {
            const PreyTarget& other = pair.second;
            if (other.name != target.name && other.is_alive) {
                double dx = target.pose.x - other.pose.x;
                double dy = target.pose.y - other.pose.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                min_distance_to_others = std::min(min_distance_to_others, distance);
            }
        }
        
        return min_distance_to_others > isolation_threshold_;
    }
    
    double calculate_threat_score(const PreyTarget& target)
    {
        // Simple threat score based on distance and isolation
        double distance_threat = 1.0 / (1.0 + target.distance);
        double isolation_threat = target.is_isolated ? 0.8 : 0.3;
        
        return distance_threat * isolation_threat;
    }
    
    void hunt_target(geometry_msgs::msg::Twist& cmd_vel, const PreyTarget& target)
    {
        // Calculate angle to target
        double dx = target.pose.x - predator_pose_.x;
        double dy = target.pose.y - predator_pose_.y;
        double target_angle = std::atan2(dy, dx);
        
        // Calculate angle difference
        double angle_diff = target_angle - predator_pose_.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        // Check if target is caught
        if (target.distance <= catch_range_) {
            catch_prey(target);
            return;
        }
        
        // Determine speed based on distance and energy
        double linear_speed;
        is_sprinting_ = false;
        
        if (target.distance <= attack_range_) {
            // Very close - decide whether to sprint based on energy
            if (current_energy_ > 40.0) {
                linear_speed = sprint_speed_;
                is_sprinting_ = true;
                RCLCPP_DEBUG(this->get_logger(), "SPRINT ATTACK %s - Distance: %.2f, Energy: %.1f", 
                           target.name.c_str(), target.distance, current_energy_);
            } else {
                linear_speed = hunting_speed_;
                RCLCPP_DEBUG(this->get_logger(), "NORMAL ATTACK %s - Distance: %.2f, Energy: %.1f", 
                           target.name.c_str(), target.distance, current_energy_);
            }
        } else if (target.distance <= stalking_range_) {
            // Within stalking range - hunting speed
            linear_speed = hunting_speed_;
            RCLCPP_DEBUG(this->get_logger(), "HUNTING %s - Distance: %.2f, Energy: %.1f", 
                       target.name.c_str(), target.distance, current_energy_);
        } else {
            // Far away - energy-efficient stalking
            linear_speed = stalking_speed_;
            RCLCPP_DEBUG(this->get_logger(), "STALKING %s - Distance: %.2f, Energy: %.1f", 
                       target.name.c_str(), target.distance, current_energy_);
        }
        
        // Set velocities
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = 0.8 * angle_diff; // Responsive turning
        
        // Log hunting status occasionally
        static int hunt_log_counter = 0;
        if (++hunt_log_counter % 100 == 0) { // Every 10 seconds
            RCLCPP_INFO(this->get_logger(), "Hunting %s - Distance: %.2f, Efficiency: %.3f, Score: %.3f", 
                       target.name.c_str(), target.distance, target.energy_efficiency, 
                       calculate_target_score(target));
        }
    }
    
    void rest_behavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        if (!rest_position_set_) {
            // Set rest position if not already set
            rest_position_ = predator_pose_;
            rest_position_set_ = true;
        }
        
<<<<<<< HEAD
        // Calculate distance from rest position
        double dx = rest_position_.x - predator_pose_.x;
        double dy = rest_position_.y - predator_pose_.y;
        double distance_from_rest = std::sqrt(dx*dx + dy*dy);
        
        if (distance_from_rest > 0.5) {
            // Move back to rest position if we've drifted too far
            double target_angle = std::atan2(dy, dx);
            double angle_diff = target_angle - predator_pose_.theta;
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
            
            cmd_vel.linear.x = 0.1;  // Very slow movement back to rest position
            cmd_vel.angular.z = 0.5 * angle_diff;
            
            static int return_log_counter = 0;
            if (++return_log_counter % 200 == 0) {  // Log every 20 seconds
                RCLCPP_INFO(this->get_logger(), "Returning to rest position - Distance: %.2f", distance_from_rest);
            }
        } else {
            // Stay still at rest position
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            
            static int rest_log_counter = 0;
            if (++rest_log_counter % 200 == 0) {  // Log every 20 seconds
                RCLCPP_INFO(this->get_logger(), "Resting at position (%.1f, %.1f) - Energy: %.1f/%.1f (%.1f%%) - Duration: %.1fs", 
                           rest_position_.x, rest_position_.y, current_energy_, max_energy_, 
                           (current_energy_/max_energy_)*100, rest_duration_);
            }
=======
        static int rest_log_counter = 0;
        if (++rest_log_counter % 200 == 0) {  // Log every 20 seconds
            RCLCPP_INFO(this->get_logger(), "RESTING - Energy too low (%.1f/%.1f), recovering...", 
                       current_energy_, max_energy_);
>>>>>>> dd0c26786ce801157d5cad41355f269985dc39bb
        }
    }
    
    void patrol_behavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Energy-efficient patrol behavior when no prey detected
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
            RCLCPP_INFO(this->get_logger(), "Patrolling - No prey detected, Energy: %.1f", current_energy_);
        }
    }
    
    void catch_prey(const PreyTarget& target)
    {
<<<<<<< HEAD
                    RCLCPP_INFO(this->get_logger(), "PREY CAUGHT! %s at distance %.2f, Energy remaining: %.1f", 
=======
        RCLCPP_INFO(this->get_logger(), "PREY CAUGHT! %s at distance %.2f, Energy remaining: %.1f", 
>>>>>>> dd0c26786ce801157d5cad41355f269985dc39bb
                   target.name.c_str(), target.distance, current_energy_);
        
        // Mark prey as dead
        prey_targets_[target.name].is_alive = false;
        
        // Kill the prey
        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_request->name = target.name;
        
        kill_futures_.push_back(kill_client_->async_send_request(kill_request).future.share());
        pending_kills_[kill_futures_.size() - 1] = target.name;
        
        // Reset current target
        current_target_ = nullptr;
    }
    
    void check_kill_completion()
    {
        for (size_t i = 0; i < kill_futures_.size(); ++i) {
            if (kill_futures_[i].wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                try {
                    auto result = kill_futures_[i].get();
                    std::string prey_name = pending_kills_[i];
                    RCLCPP_INFO(this->get_logger(), "%s killed successfully", prey_name.c_str());
                    
                    // Check if all prey are dead
                    int alive_prey = 0;
                    for (const auto& pair : prey_targets_) {
                        if (pair.second.is_alive) {
                            alive_prey++;
                        }
                    }
                    
                    if (alive_prey == 0) {
                        RCLCPP_INFO(this->get_logger(), "MISSION ACCOMPLISHED! All prey eliminated!");
                        RCLCPP_INFO(this->get_logger(), "Final Energy: %.1f/%.1f (%.1f%%)", 
                                   current_energy_, max_energy_, (current_energy_/max_energy_)*100);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "%d prey remaining...", alive_prey);
                    }
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to kill prey: %s", e.what());
                }
                
                // Remove completed future
                kill_futures_.erase(kill_futures_.begin() + i);
                pending_kills_.erase(i);
                break; // Exit loop since we modified the vector
            }
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
    
    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<turtlesim::msg::Color>::SharedPtr pen_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr predator_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_2_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_3_;
    rclcpp::TimerBase::SharedPtr movement_timer_;
    rclcpp::TimerBase::SharedPtr energy_timer_;
    rclcpp::TimerBase::SharedPtr kill_check_timer_;
    
    // Service clients
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    
    // Kill tracking
    std::vector<rclcpp::Client<turtlesim::srv::Kill>::SharedFuture> kill_futures_;
    std::map<size_t, std::string> pending_kills_;
    
    // State variables
    turtlesim::msg::Pose predator_pose_;
    std::map<std::string, PreyTarget> prey_targets_;
    PreyTarget* current_target_;
    bool predator_pose_received_;
    bool is_sprinting_;
    int target_switch_counter_;
    double current_linear_velocity_;
    
    // Energy system
    double max_energy_;
    double current_energy_;
    double energy_drain_rate_;
    double energy_recovery_rate_;
    double min_energy_to_hunt_;
    double sprint_energy_cost_;
    
    // Rest system
    bool is_resting_;
    double rest_energy_threshold_;
    double rest_recovery_threshold_;
    double rest_duration_;
    double rest_start_time_;
    turtlesim::msg::Pose rest_position_;
    bool rest_position_set_;
    
    // Hunting parameters
    double detection_range_;
    double stalking_range_;
    double attack_range_;
    double catch_range_;
    double isolation_threshold_;
    
    // Speed parameters
    double stalking_speed_;
    double hunting_speed_;
    double sprint_speed_;
    
    // Target selection weights
    double distance_weight_;
    double efficiency_weight_;
    double isolation_weight_;
    double probability_weight_;
    
    // Border limits
    double border_min_;
    double border_max_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EnergyEfficientPredatorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 