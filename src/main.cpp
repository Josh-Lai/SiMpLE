#include <chrono>
#include <filesystem>

#include "Map.hpp"
#include "Register.hpp"
#include "Scan.hpp"
#include "utils.hpp"

#include "rclcpp/rclcpp.hpp"

class Simple : public rclcpp::Node {
public:
  Simple() : Node("simple") { RCLCPP_INFO(this->get_logger(), "Started simple"); }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simple>());
  rclcpp::shutdown();
  return 0;
}
