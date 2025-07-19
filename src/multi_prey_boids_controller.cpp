#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <vector>
#include <cmath>
#include <random>

class MultiPreyBoidsController : public rclcpp::Node
{
public:
    MultiPreyBoidsController() : Node("multi_prey_boids_controller")
    {
        // Get prey name from parameter
        this->declare_parameter("prey_name", "prey_turtle_1");
        prey_name_ = this->get_parameter("prey_name").as_string();
        
        // Create publisher for prey velocity commands
        std::string cmd_vel_topic = "/" + prey_name_ + "/cmd_vel";
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        
        // Create subscriber for prey pose
        std::string prey_pose_topic = "/" + prey_name_ + "/pose";
        prey_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            prey_pose_topic, 10,
            std::bind(&MultiPreyBoidsController::prey_pose_callback, this, std::placeholders::_1)
        );
        
        // Create subscriber for predator pose
        predator_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/predator_turtle/pose", 10,
            std::bind(&MultiPreyBoidsController::predator_pose_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to other prey poses for flocking behavior
        prey_pose_sub_1_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle_1/pose", 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                if (prey_name_ != "prey_turtle_1") {
                    other_prey_poses_["prey_turtle_1"] = *msg;
                }
            }
        );
        
        prey_pose_sub_2_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle_2/pose", 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                if (prey_name_ != "prey_turtle_2") {
                    other_prey_poses_["prey_turtle_2"] = *msg;
                }
            }
        );
        
        prey_pose_sub_3_ = this->create_subscription<turtlesim::msg::Pose>(
            "/prey_turtle_3/pose", 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                if (prey_name_ != "prey_turtle_3") {
                    other_prey_poses_["prey_turtle_3"] = *msg;
                }
            }
        );
        
        // Boids parameters - optimized for multi-prey
        separation_weight_ = 0.15;     // Weight for separation rule
        alignment_weight_ = 0.1;       // Weight for alignment rule
        cohesion_weight_ = 0.1;        // Weight for cohesion rule
        predator_avoidance_weight_ = 0.25; // Weight for predator avoidance
        border_avoidance_weight_ = 0.1; // Weight for border avoidance
        
        // Flocking parameters - appropriate for turtlesim (11x11 space)
        neighbor_radius_ = 3.0;        // Radius to consider neighbors
        separation_radius_ = 1.5;      // Minimum distance between prey
        predator_detection_radius_ = 4.0; // Distance to detect predator
        
        // Movement parameters
        max_speed_ = 1.2;              // Maximum speed (fleeing speed)
        min_speed_ = 0.4;              // Minimum speed (normal movement)
        max_turn_rate_ = 0.6;          // Maximum turning rate
        
        // Border limits
        border_min_ = 0.5;
        border_max_ = 10.5;
        
        // Initialize poses and state
        prey_pose_received_ = false;
        predator_pose_received_ = false;
        other_prey_poses_.clear();
        
        // Initialize random number generator for noise
        random_engine_ = std::mt19937(std::random_device{}());
        noise_dist_ = std::uniform_real_distribution<double>(-0.02, 0.02);
        
        // Start movement timer
        movement_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update every 100ms
            std::bind(&MultiPreyBoidsController::update_movement, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Multi-Prey Boids Controller initialized for %s", prey_name_.c_str());
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
        
        // Convert force to velocity
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
        
        // Log movement occasionally
        static int debug_counter = 0;
        if (++debug_counter % 50 == 0) { // Log every 5 seconds
            RCLCPP_DEBUG(this->get_logger(), "🦌 %s - Speed: %.2f, Turn: %.2f", 
                       prey_name_.c_str(), cmd_vel.linear.x, cmd_vel.angular.z);
        }
        
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    Vector2D calculate_separation()
    {
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
        
        // Avoid getting too close to other prey
        for (const auto& pair : other_prey_poses_) {
            const turtlesim::msg::Pose& other_pose = pair.second;
            double dx = prey_pose_.x - other_pose.x;
            double dy = prey_pose_.y - other_pose.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < separation_radius_ && distance > 0.0) {
                // Stronger separation for closer prey
                double separation_strength = (separation_radius_ - distance) / separation_radius_;
                separation_force.x += (dx / distance) * separation_strength;
                separation_force.y += (dy / distance) * separation_strength;
            }
        }
        
        return separation_force;
    }
    
    Vector2D calculate_alignment()
    {
        Vector2D alignment_force(0.0, 0.0);
        int neighbor_count = 0;
        
        // Align with nearby prey
        for (const auto& pair : other_prey_poses_) {
            const turtlesim::msg::Pose& other_pose = pair.second;
            double dx = other_pose.x - prey_pose_.x;
            double dy = other_pose.y - prey_pose_.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < neighbor_radius_ && distance > 0.0) {
                // Add velocity contribution from this neighbor
                alignment_force.x += std::cos(other_pose.theta);
                alignment_force.y += std::sin(other_pose.theta);
                neighbor_count++;
            }
        }
        
        // Average the alignment force
        if (neighbor_count > 0) {
            alignment_force.x /= neighbor_count;
            alignment_force.y /= neighbor_count;
        }
        
        return alignment_force;
    }
    
    Vector2D calculate_cohesion()
    {
        Vector2D cohesion_force(0.0, 0.0);
        int neighbor_count = 0;
        
        // Move toward center of nearby prey
        for (const auto& pair : other_prey_poses_) {
            const turtlesim::msg::Pose& other_pose = pair.second;
            double dx = other_pose.x - prey_pose_.x;
            double dy = other_pose.y - prey_pose_.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < neighbor_radius_ && distance > 0.0) {
                cohesion_force.x += dx;
                cohesion_force.y += dy;
                neighbor_count++;
            }
        }
        
        // Average the cohesion force
        if (neighbor_count > 0) {
            cohesion_force.x /= neighbor_count;
            cohesion_force.y /= neighbor_count;
        }
        
        return cohesion_force;
    }
    
    Vector2D calculate_predator_avoidance()
    {
        Vector2D avoidance_force(0.0, 0.0);
        
        if (predator_pose_received_) {
            double dx = prey_pose_.x - predator_pose_.x;
            double dy = prey_pose_.y - predator_pose_.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < predator_detection_radius_ && distance > 0.0) {
                // Stronger avoidance for closer predator
                double avoidance_strength = (predator_detection_radius_ - distance) / predator_detection_radius_;
                avoidance_force.x = (dx / distance) * avoidance_strength;
                avoidance_force.y = (dy / distance) * avoidance_strength;
            }
        }
        
        return avoidance_force;
    }
    
    Vector2D calculate_border_avoidance()
    {
        Vector2D border_force(0.0, 0.0);
        
        // Calculate distance to center
        double dx = 5.5 - prey_pose_.x;
        double dy = 5.5 - prey_pose_.y;
        double distance_to_center = std::sqrt(dx*dx + dy*dy);
        
        // If too close to borders, move toward center
        if (prey_pose_.x < border_min_ + 1.0 || prey_pose_.x > border_max_ - 1.0 ||
            prey_pose_.y < border_min_ + 1.0 || prey_pose_.y > border_max_ - 1.0) {
            border_force.x = dx / distance_to_center;
            border_force.y = dy / distance_to_center;
        }
        
        return border_force;
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr predator_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_2_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr prey_pose_sub_3_;
    rclcpp::TimerBase::SharedPtr movement_timer_;
    
    // State variables
    turtlesim::msg::Pose prey_pose_;
    turtlesim::msg::Pose predator_pose_;
    std::map<std::string, turtlesim::msg::Pose> other_prey_poses_;
    bool prey_pose_received_;
    bool predator_pose_received_;
    std::string prey_name_;
    
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
    
    // Random number generation
    std::mt19937 random_engine_;
    std::uniform_real_distribution<double> noise_dist_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiPreyBoidsController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 