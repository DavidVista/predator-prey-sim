#include <string>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class Spawner : public rclcpp::Node {
public:
  Spawner() : Node("spawner") {
    start_srv_ = create_service<std_srvs::srv::Empty>(
      "/start",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
             std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        reset_world();
        spawn_turtles();
      });

    kill_client = this->create_client<turtlesim::srv::Kill>("/kill");
    spawn_client = this->create_client<turtlesim::srv::Spawn>("/spawn");
  }

private:
  void reset_world() {
    auto request_to_kill = std::make_shared<turtlesim::srv::Kill::Request>();
    request_to_kill->name = "turtle1";

    while (!kill_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services. Exiting");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again");
    }

    auto result_kill = kill_client->async_send_request(request_to_kill);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result_kill) ==
        rclcpp::FutureReturnCode::SUCCESS
    ) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully sent kill request");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to send kill request");
    }

    while (!spawn_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for services. Exiting");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again");
    }
  }

  void spawn_turtles() {
    for (int i = 0; i < NUMBER_OF_PREY; ++i) {
        auto request_to_spawn = std::make_shared<turtlesim::srv::Spawn::Request>();
        request_to_spawn->x = std::rand() * 11.0;
        request_to_spawn->y = std::rand() * 11.0;
        request_to_spawn->theta = std::rand() * 2 * M_PI - M_PI;
        request_to_spawn->name = std::string("prey_turtle") + std::to_string(i+1);
        auto result_spawn = spawn_client->async_send_request(request_to_spawn);

        if (rclcpp::spin_until_future_complete(shared_from_this(), result_spawn) ==
            rclcpp::FutureReturnCode::SUCCESS
        ) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully sent spawn request");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to send spawn request");
        }
    }
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
  int NUMBER_OF_PREY = 5;
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Spawner>());
  rclcpp::shutdown();
  return 0;
}
