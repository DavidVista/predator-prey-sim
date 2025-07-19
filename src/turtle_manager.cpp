#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include <random>

class TurtleManager : public rclcpp::Node
{
public:
    TurtleManager() : Node("turtle_manager")
    {
        // Declare parameter for single vs multi-prey mode
        this->declare_parameter("multi_prey_mode", true);
        multi_prey_mode_ = this->get_parameter("multi_prey_mode").as_bool();
        
        // Create service clients
        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        
        // Initialize random number generator for spawn positions
        random_engine_ = std::mt19937(std::random_device{}());
        spawn_x_dist_ = std::uniform_real_distribution<double>(2.0, 9.0);
        spawn_y_dist_ = std::uniform_real_distribution<double>(2.0, 9.0);
        
        if (multi_prey_mode_) {
            RCLCPP_INFO(this->get_logger(), "Turtle Manager initialized - Multi-Prey Mode (3 prey)");
        } else {
            RCLCPP_INFO(this->get_logger(), "Turtle Manager initialized - Single-Prey Mode (1 prey)");
        }
        
        // Start the turtle management process
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TurtleManager::manage_turtles, this)
        );
    }

private:
    void manage_turtles()
    {
        // Only run once
        timer_->cancel();
        
        // First, kill the default turtle
        kill_default_turtle();
    }
    
    void kill_default_turtle()
    {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = "turtle1";
        
        RCLCPP_INFO(this->get_logger(), "Attempting to kill default turtle...");
        
        auto future = kill_client_->async_send_request(
            request,
            std::bind(&TurtleManager::kill_callback, this, std::placeholders::_1)
        );
    }
    
    void kill_callback(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Default turtle killed successfully");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to kill default turtle: %s", e.what());
        }
        
        // Wait a bit, then spawn our prey turtles
        spawn_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TurtleManager::spawn_prey_turtles, this)
        );
    }
    
    void spawn_prey_turtles()
    {
        // Only run once
        spawn_timer_->cancel();
        
        if (multi_prey_mode_) {
            // Multi-prey mode: spawn 3 prey turtles
            std::vector<std::string> prey_names = {"prey_turtle_1", "prey_turtle_2", "prey_turtle_3"};
            std::vector<std::pair<double, double>> spawn_positions = {
                {3.0, 3.0},  // Bottom left
                {7.0, 7.0},  // Top right  
                {5.0, 5.0}   // Center
            };
            
            for (size_t i = 0; i < prey_names.size(); ++i) {
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                request->x = spawn_positions[i].first;
                request->y = spawn_positions[i].second;
                request->theta = 0.0;
                request->name = prey_names[i];
                
                RCLCPP_INFO(this->get_logger(), "Spawning %s at (%.1f, %.1f)...", 
                           prey_names[i].c_str(), request->x, request->y);
                
                spawn_futures_.push_back(spawn_client_->async_send_request(request).future.share());
            }
        } else {
            // Single-prey mode: spawn 1 prey turtle
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = 5.0;
            request->y = 5.0;
            request->theta = 0.0;
            request->name = "prey_turtle";
            
            RCLCPP_INFO(this->get_logger(), "Spawning prey_turtle at (%.1f, %.1f)...", request->x, request->y);
            
            spawn_futures_.push_back(spawn_client_->async_send_request(request).future.share());
        }
        
        // Start a timer to check when all spawns are complete
        spawn_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleManager::check_spawn_completion, this)
        );
    }
    
    void check_spawn_completion()
    {
        bool all_complete = true;
        int completed_count = 0;
        
        for (auto& future : spawn_futures_) {
            if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                try {
                    auto result = future.get();
                    completed_count++;
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to spawn prey: %s", e.what());
                }
            } else {
                all_complete = false;
            }
        }
        
        if (all_complete) {
            spawn_check_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "All %d prey spawned successfully!", completed_count);
            
            // Start timer to spawn predator after 3 seconds
            predator_timer_ = this->create_wall_timer(
                std::chrono::seconds(3),
                std::bind(&TurtleManager::spawn_predator_turtle, this)
            );
        }
    }
    
    void spawn_predator_turtle()
    {
        // Only run once
        predator_timer_->cancel();
        
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 1.0;
        request->y = 1.0;
        request->theta = 0.0;
        request->name = "predator_turtle";
        
        RCLCPP_INFO(this->get_logger(), "Spawning predator turtle at (%.1f, %.1f)...", request->x, request->y);
        
        auto future = spawn_client_->async_send_request(
            request,
            std::bind(&TurtleManager::spawn_predator_callback, this, std::placeholders::_1)
        );
    }
    
    void spawn_predator_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Predator turtle spawned successfully!");
            if (multi_prey_mode_) {
                RCLCPP_INFO(this->get_logger(), "Multi-prey simulation ready - 3 prey vs 1 energy-efficient predator!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Single-prey simulation ready - 1 prey vs 1 predator!");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn predator turtle: %s", e.what());
        }
    }
    
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    rclcpp::TimerBase::SharedPtr spawn_check_timer_;
    rclcpp::TimerBase::SharedPtr predator_timer_;
    
    // Multi-prey tracking
    std::vector<rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture> spawn_futures_;
    
    // Random number generation for spawn positions
    std::mt19937 random_engine_;
    std::uniform_real_distribution<double> spawn_x_dist_;
    std::uniform_real_distribution<double> spawn_y_dist_;
    
    // Parameter for multi-prey mode
    bool multi_prey_mode_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 