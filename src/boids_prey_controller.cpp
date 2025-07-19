#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <vector>
#include <cmath>
#include <random>

class BoidsPreyController : public rclcpp::Node
{
public:
    BoidsPreyController() : Node("boids_prey_controller")
    {
        // Create publisher for prey velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/prey_turtle/cmd_vel", 10);
        
        // Create subscriber for prey pose
        prey_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle/pose", 10,
            std::bind(&BoidsPreyController::prey_pose_callback, this, std::placeholders::_1)
        );
        
        // Create subscriber for predator pose
        predator_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/predator_turtle/pose", 10,
            std::bind(&BoidsPreyController::predator_pose_callback, this, std::placeholders::_1)
        );
        
        // Boids parameters - REDUCED for slow movement
        separation_weight_ = 0.1;      // Weight for separation rule (reduced)
        alignment_weight_ = 0.1;       // Weight for alignment rule (reduced)
        cohesion_weight_ = 0.1;        // Weight for cohesion rule (reduced)
        predator_avoidance_weight_ = 0.2; // Weight for predator avoidance (reduced)
        
        // Flocking parameters - Appropriate for turtlesim (11x11 space)
        neighbor_radius_ = 2.0;        // Radius to consider neighbors
        separation_radius_ = 1.0;      // Minimum distance between prey
        predator_detection_radius_ = 3.0; // Distance to detect predator
        
        // Movement parameters
        max_speed_ = 1.0;               // Increased maximum speed (antelope fleeing speed)
        min_speed_ = 0.3;               // Increased minimum speed (antelope normal speed)
        max_turn_rate_ = 0.5;           // Maximum turning rate
        
        // Border limits
        border_min_ = 0.5;
        border_max_ = 10.5;
        border_avoidance_weight_ = 0.1; // Weight for border avoidance (reduced)
        
        // Initialize poses and state
        prey_pose_received_ = false;
        predator_pose_received_ = false;
        
        // Initialize random number generator for noise
        random_engine_ = std::mt19937(std::random_device{}());
        noise_dist_ = std::uniform_real_distribution<double>(-0.01, 0.01); // Reduced noise
        
        // Start movement timer
        movement_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update every 100ms
            std::bind(&BoidsPreyController::update_movement, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Boids Prey Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Neighbor radius: %.1f, Separation radius: %.1f, Predator detection: %.1f", 
                   neighbor_radius_, separation_radius_, predator_detection_radius_);
    }

private:
    struct Vector2D {
        double x, y;
        Vector2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
        
        Vector2D operator+(const Vector2D& other) const {
            return Vector2D(x + other.x, y + other.y);
        }
        
        Vector2D operator-(const Vector2D& other) const {
            return Vector2D(x - other.x, y - other.y);
        }
        
        Vector2D operator*(double scalar) const {
            return Vector2D(x * scalar, y * scalar);
        }
        
        double magnitude() const {
            return std::sqrt(x*x + y*y);
        }
        
        Vector2D normalize() const {
            double mag = magnitude();
            if (mag > 0.0) {
                return Vector2D(x/mag, y/mag);
            }
            return Vector2D(0.0, 0.0);
        }
    };
    
    void prey_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        prey_pose_ = *msg;
        prey_pose_received_ = true;
    }
    
    void predator_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        predator_pose_ = *msg;
        predator_pose_received_ = true;
    }
    
    void update_movement()
    {
        if (!prey_pose_received_) {
            return;  // Wait for prey pose
        }
        
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        // Calculate Boids behavior
        Vector2D separation_force = calculate_separation();
        Vector2D alignment_force = calculate_alignment();
        Vector2D cohesion_force = calculate_cohesion();
        Vector2D predator_avoidance_force = calculate_predator_avoidance();
        Vector2D border_avoidance_force = calculate_border_avoidance();
        
        // Combine all forces
        Vector2D total_force = separation_force * separation_weight_ +
                              alignment_force * alignment_weight_ +
                              cohesion_force * cohesion_weight_ +
                              predator_avoidance_force * predator_avoidance_weight_ +
                              border_avoidance_force * border_avoidance_weight_;
        
        // Add some noise for more natural movement
        Vector2D noise(noise_dist_(random_engine_), noise_dist_(random_engine_));
        total_force = total_force + noise;
        
        // Convert force to velocity - FORCE EXACT SPEED
        double force_magnitude = total_force.magnitude();
        if (force_magnitude > 0.0) {
            Vector2D desired_direction = total_force.normalize();
            
            // Calculate speed based on force magnitude and predator presence
            double desired_speed;
            if (predator_pose_received_) {
                double dx = predator_pose_.x - prey_pose_.x;
                double dy = predator_pose_.y - prey_pose_.y;
                double distance_to_predator = std::sqrt(dx*dx + dy*dy);
                
                if (distance_to_predator < predator_detection_radius_) {
                    desired_speed = max_speed_; // Flee at max speed when predator detected
                } else {
                    desired_speed = min_speed_; // Normal speed when safe
                }
            } else {
                desired_speed = min_speed_; // Normal speed when no predator info
            }
            
            // Calculate angle to desired direction
            double desired_angle = std::atan2(desired_direction.y, desired_direction.x);
            double current_angle = prey_pose_.theta;
            
            // Calculate angle difference
            double angle_diff = desired_angle - current_angle;
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
            
            // Set velocities with realistic speeds
            cmd_vel.linear.x = desired_speed;
            cmd_vel.angular.z = std::clamp(angle_diff, -max_turn_rate_, max_turn_rate_);
    } else {
        // No force - maintain current direction with minimum speed
        cmd_vel.linear.x = min_speed_;
        cmd_vel.angular.z = 0.0;
    }
    
    // DEBUG: Log the actual velocity being published
    static int debug_counter = 0;
    if (++debug_counter % 10 == 0) { // Log every second
        RCLCPP_INFO(this->get_logger(), "Boids Prey - Published velocity: linear.x=%.4f, angular.z=%.4f, max_speed_=%.4f", 
                   cmd_vel.linear.x, cmd_vel.angular.z, max_speed_);
    }
    
    cmd_vel_pub_->publish(cmd_vel);
    }
    
    Vector2D calculate_separation()
    {
        // For single prey, separation is based on border avoidance
        // In a multi-prey system, this would consider other prey positions
        Vector2D separation_force(0.0, 0.0);
        
        // Avoid getting too close to borders
        if (prey_pose_.x < border_min_ + separation_radius_) {
            separation_force.x += 1.0;
        }
        if (prey_pose_.x > border_max_ - separation_radius_) {
            separation_force.x -= 1.0;
        }
        if (prey_pose_.y < border_min_ + separation_radius_) {
            separation_force.y += 1.0;
        }
        if (prey_pose_.y > border_max_ - separation_radius_) {
            separation_force.y -= 1.0;
        }
        
        return separation_force;
    }
    
    Vector2D calculate_alignment()
    {
        // For single prey, alignment maintains current direction
        // In a multi-prey system, this would align with neighbors' directions
        Vector2D alignment_force(0.0, 0.0);
        
        // Maintain current direction with slight variations
        alignment_force.x = std::cos(prey_pose_.theta);
        alignment_force.y = std::sin(prey_pose_.theta);
        
        return alignment_force;
    }
    
    Vector2D calculate_cohesion()
    {
        // For single prey, cohesion moves toward center of safe area
        // In a multi-prey system, this would move toward center of neighbors
        Vector2D cohesion_force(0.0, 0.0);
        
        // Move toward center of the map (safe area)
        double center_x = (border_min_ + border_max_) / 2.0;
        double center_y = (border_min_ + border_max_) / 2.0;
        
        cohesion_force.x = center_x - prey_pose_.x;
        cohesion_force.y = center_y - prey_pose_.y;
        
        // Normalize to avoid overwhelming other forces
        double magnitude = cohesion_force.magnitude();
        if (magnitude > 0.0) {
            cohesion_force = cohesion_force * (1.0 / magnitude);
        }
        
        return cohesion_force;
    }
    
    Vector2D calculate_predator_avoidance()
    {
        Vector2D avoidance_force(0.0, 0.0);
        
        if (!predator_pose_received_) {
            return avoidance_force;
        }
        
        // Calculate distance to predator
        double dx = prey_pose_.x - predator_pose_.x;
        double dy = prey_pose_.y - predator_pose_.y;
        double distance_to_predator = std::sqrt(dx*dx + dy*dy);
        
        // If predator is within detection radius, flee
        if (distance_to_predator <= predator_detection_radius_) {
            // Stronger avoidance when predator is closer
            double avoidance_strength = 1.0 - (distance_to_predator / predator_detection_radius_);
            avoidance_strength = std::max(0.1, avoidance_strength); // Minimum avoidance
            
            avoidance_force.x = dx * avoidance_strength;
            avoidance_force.y = dy * avoidance_strength;
            
            // Log avoidance behavior occasionally
            static int avoidance_log_counter = 0;
            if (++avoidance_log_counter % 50 == 0) {  // Log every 5 seconds
                RCLCPP_INFO(this->get_logger(), "FLEEING from predator - Distance: %.2f, Strength: %.2f", 
                           distance_to_predator, avoidance_strength);
            }
        }
        
        return avoidance_force;
    }
    
    Vector2D calculate_border_avoidance()
    {
        Vector2D border_force(0.0, 0.0);
        
        // Calculate distance to borders
        double dist_to_left = prey_pose_.x - border_min_;
        double dist_to_right = border_max_ - prey_pose_.x;
        double dist_to_bottom = prey_pose_.y - border_min_;
        double dist_to_top = border_max_ - prey_pose_.y;
        
        // Avoid borders when getting close
        double border_threshold = 2.0;
        
        if (dist_to_left < border_threshold) {
            border_force.x += (border_threshold - dist_to_left) / border_threshold;
        }
        if (dist_to_right < border_threshold) {
            border_force.x -= (border_threshold - dist_to_right) / border_threshold;
        }
        if (dist_to_bottom < border_threshold) {
            border_force.y += (border_threshold - dist_to_bottom) / border_threshold;
        }
        if (dist_to_top < border_threshold) {
            border_force.y -= (border_threshold - dist_to_top) / border_threshold;
        }
        
        return border_force;
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr predator_pose_sub_;
    rclcpp::TimerBase::SharedPtr movement_timer_;
    
    turtlesim::msg::Pose prey_pose_;
    turtlesim::msg::Pose predator_pose_;
    bool prey_pose_received_;
    bool predator_pose_received_;
    
    // Boids parameters
    double separation_weight_;
    double alignment_weight_;
    double cohesion_weight_;
    double predator_avoidance_weight_;
    double border_avoidance_weight_;
    
    // Flocking parameters
    double neighbor_radius_;
    double separation_radius_;
    double predator_detection_radius_;
    
    // Movement parameters
    double max_speed_;
    double min_speed_;
    double max_turn_rate_;
    
    // Border limits
    double border_min_;
    double border_max_;
    
    // Random number generation for noise
    std::mt19937 random_engine_;
    std::uniform_real_distribution<double> noise_dist_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoidsPreyController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 