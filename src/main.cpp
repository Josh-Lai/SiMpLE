#include <chrono>
#include <filesystem>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

#include "ConfigParser.hpp"
#include "Map.hpp"
#include "Register.hpp"
#include "Scan.hpp"
#include "utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class Simple : public rclcpp::Node {
public:
  Simple() : Node("simple") {
    RCLCPP_INFO(this->get_logger(), "Started simple");
    config_.verbose = this->declare_parameter("verbose", false);
    config_.kitti = this->declare_parameter("kitti", false);
    config_.sigma = this->declare_parameter("sigma", 0.25);
    config_.rMap = this->declare_parameter("rMap", 1.0);
    config_.rNew = this->declare_parameter("rNew", 0.3);
    config_.convergenceTol = this->declare_parameter("convergenceTol", 1e-3);
    config_.maxSensorRange = this->declare_parameter("maxSensorRange", 120.0);
    config_.minSensorRange = this->declare_parameter("minSensorRange", 10.0);
    config_.scanPath = this->declare_parameter("scanPath", "");
    config_.outputFileName = this->declare_parameter("outputFileName", "");
    config_.useLivePointCloud =
        this->declare_parameter("useLivePointCloud", true);
    config_.pointCloudTopic = this->declare_parameter("pointCloudTopic", "");

    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.pointCloudTopic, rclcpp::QoS(10).keep_all(),
        std::bind(&Simple::run_simple, this, _1));
    subMap_ = Map(config_);
    scanToMapRegister_ = Register(config_);
  }

private:
  ConfigParser config_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  Map subMap_;
  Register scanToMapRegister_;

  void run_simple(sensor_msgs::msg::PointCloud2 msg) {
    // Convert the pointcloud into data structure seen in the original
    // implementation
    Scan newScan(config_);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simple>());
  rclcpp::shutdown();
  return 0;
}
