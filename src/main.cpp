#include <chrono>
#include <filesystem>
#include <oneapi/tbb/parallel_for.h>
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
    config_.mapFrame = this->declare_parameter("mapFrame", "");

    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.pointCloudTopic, rclcpp::QoS(10).keep_all(),
        std::bind(&Simple::run_simple, this, _1));
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_points", rclcpp::QoS(10));
    subMap_ = Map(config_);
    scanToMapRegister_ = Register(config_);
  }

private:
  ConfigParser config_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  Map subMap_;
  Register scanToMapRegister_;
  int scanCount_ = 0;

  void run_simple(sensor_msgs::msg::PointCloud2 msg) {
    // Convert the pointcloud into data structure seen in the original
    // implementation
    Scan newScan(config_);
    newScan.readScan(msg);
    RCLCPP_INFO(this->get_logger(), "Read Scan with %d points",
                (int)newScan.ptCloud.size());
    // Process as per the SiMPLE algorithm
    Eigen::Matrix4d hypothesis = Eigen::Matrix4d::Identity();
    if (scanCount_ > 0) {
      scanToMapRegister_.registerScan(newScan.ptCloud, subMap_.pcForKdTree_);
    }
    hypothesis = homogeneous(
        scanToMapRegister_.regResult(0), scanToMapRegister_.regResult(1),
        scanToMapRegister_.regResult(2), scanToMapRegister_.regResult(3),
        scanToMapRegister_.regResult(4), scanToMapRegister_.regResult(5));
    subMap_.updateMap(newScan.ptCloud, hypothesis);
    scanCount_++;

    // Output the result
    pcl::PointCloud<pcl::PointXYZ> pclMap = simple_to_pcl(subMap_.ptCloud);
    sensor_msgs::msg::PointCloud2 outMap;
    pcl::toROSMsg(pclMap, outMap);
    outMap.header.stamp = this->get_clock()->now();
    outMap.header.frame_id = config_.mapFrame;
    map_pub_->publish(outMap);
  }

  pcl::PointCloud<pcl::PointXYZ>
  simple_to_pcl(std::vector<Eigen::Vector4d> ptCloud) {
    // create pcl point cloud from simple
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pclCloud.resize(ptCloud.size());
    // Loop over SiMPLE point cloud to populate
    tbb::parallel_for(tbb::blocked_range<int>(0, ptCloud.size()),
                      [&](tbb::blocked_range<int> r) {
                        for (unsigned int i = r.begin(); i < r.end(); i++) {
                          pclCloud.points[i].x = ptCloud[i][0];
                          pclCloud.points[i].y = ptCloud[i][1];
                          pclCloud.points[i].z = ptCloud[i][2];
                        }
                      });
    return pclCloud;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simple>());
  rclcpp::shutdown();
  return 0;
}
