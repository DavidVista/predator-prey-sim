#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

class TurtleManager : public rclcpp::Node
{
public:
    TurtleManager() : Node("turtle_manager")
    {
        // Create service clients
        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        
        RCLCPP_INFO(this->get_logger(), "Turtle Manager initialized");
        
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
        
        // Wait a bit, then spawn our custom turtle
        spawn_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TurtleManager::spawn_custom_turtle, this)
        );
    }
    
    void spawn_custom_turtle()
    {
        // Only run once
        spawn_timer_->cancel();
        
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 5.0;
        request->y = 5.0;
        request->theta = 0.0;
        request->name = "prey_turtle";
        
        RCLCPP_INFO(this->get_logger(), "Spawning prey turtle at (%.1f, %.1f)...", request->x, request->y);
        
        auto future = spawn_client_->async_send_request(
            request,
            std::bind(&TurtleManager::spawn_prey_callback, this, std::placeholders::_1)
        );
    }
    
    void spawn_prey_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Prey turtle spawned successfully!");
            
            // Start timer to spawn predator after 5 seconds
            predator_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&TurtleManager::spawn_predator_turtle, this)
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn prey turtle: %s", e.what());
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
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn predator turtle: %s", e.what());
        }
    }
    
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    rclcpp::TimerBase::SharedPtr predator_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 