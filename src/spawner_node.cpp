#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"


class Spawner : public rclcpp::Node {
public:
  Spawner() : Node("spawner") {
    // TODO: Create /start service server
    start_srv_ = create_service<std_srvs::srv::Empty>(
      "/start",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
             std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        reset_world();
        spawn_turtles();
      });
  }

private:
  void reset_world() {
    // TODO: Kill all existing turtles
  }

  void spawn_turtles() {
    // TODO: Spawn N prey turtles (random positions)
    // TODO: Spawn 1 predator turtle (center)
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Spawner>());
  rclcpp::shutdown();
  return 0;
}
