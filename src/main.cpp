#include <chrono>
#include <filesystem>
#include <oneapi/tbb/parallel_for.h>

#include "ConfigParser.hpp"
#include "Map.hpp"
#include "Register.hpp"
#include "Scan.hpp"
#include "utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pcl_ros/transforms.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/conversions.h"
#include "tf2_ros/transform_broadcaster.h"

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
    config_.pointCloudTopic = this->declare_parameter("pointCloudTopic", "");
    config_.mapFrame = this->declare_parameter("mapFrame", "");
    config_.outputFrame = this->declare_parameter("outputFrame","");
    config_.saveToFile =  this->declare_parameter("saveToFile", false);
    config_.mapFile = this->declare_parameter("mapFile", "map.pcd");
    
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.pointCloudTopic, rclcpp::QoS(10).keep_all(),
        std::bind(&Simple::run_simple, this, _1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subMap_ = Map(config_);
    scanToMapRegister_ = Register(config_);

    if(config_.saveToFile) {
      save_sub_ = this->create_subscription<std_msgs::msg::Bool> (
        "save_simple_map", rclcpp::QoS(10), std::bind(&Simple::save_map, this, _1)
      );
    }
  }


private:
  using PclPointType = pcl::PointXYZI;
  ConfigParser config_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_sub_;
  Map subMap_;
  Register scanToMapRegister_;
  int scanCount_ = 0;
  sensor_msgs::msg::PointCloud2 fullMap_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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

    // Publish the Hypothesis as a TF frame
    Eigen::Matrix3d R = hypothesis.block(0,0,3,3);
    Eigen::Quaterniond Q(R);
    geometry_msgs::msg::TransformStamped T;
    T.header.frame_id = config_.mapFrame;
    T.header.stamp = msg.header.stamp;
    if (config_.outputFrame.size() == 0) {
      T.child_frame_id = msg.header.frame_id;
    } else {
      T.child_frame_id = config_.outputFrame;
    }
    T.transform.translation.x = hypothesis(0,3);
    T.transform.translation.y = hypothesis(1,3);
    T.transform.translation.z = hypothesis(2,3);

    T.transform.rotation.x = Q.x();
    T.transform.rotation.y = Q.y();
    T.transform.rotation.z = Q.z();
    T.transform.rotation.w = Q.w();
    tf_broadcaster_->sendTransform(T);
    if (config_.saveToFile) {
      sensor_msgs::msg::PointCloud2 rosPcl;
      pcl::toROSMsg(newScan.pclPtCloud_, rosPcl);
      append_point_cloud(rosPcl, hypothesis);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>
  simple_to_pcl(std::vector<Eigen::Vector4d> ptCloud) {
    // create pcl point cloud from simple
    pcl::PointCloud<pcl::PointXYZI> pclCloud;
    pclCloud.resize(ptCloud.size());
    // Loop over SiMPLE point cloud to populate
    tbb::parallel_for(tbb::blocked_range<int>(0, ptCloud.size()),
                      [&](tbb::blocked_range<int> r) {
                        for (unsigned int i = r.begin(); i < (unsigned int)r.end(); i++) {
                          pclCloud.points[i].x = ptCloud[i][0];
                          pclCloud.points[i].y = ptCloud[i][1];
                          pclCloud.points[i].z = ptCloud[i][2];
                        }
                      });
    return pclCloud;
  }

  /**
  * Append the point cloud to the buffer (which will be saved on program exit)
  * Adds overhead, so should only be run if saveToFile is true
  */
  void append_point_cloud(sensor_msgs::msg::PointCloud2 newPcl, Eigen::Matrix4d T) {
    // Transform the point cloud to its new frame
    sensor_msgs::msg::PointCloud2 tPcl;
    pcl_ros::transformPointCloud(T.cast<float>(), newPcl, tPcl);
    pcl::concatenatePointCloud(fullMap_, tPcl, fullMap_);

  }

  /**
  * Saves the point cloud stored to a map file
  * specified in configuration
  */
  void save_map(std_msgs::msg::Bool msg) {
    if(msg.data) {
      RCLCPP_INFO(this->get_logger(), "Saving Map");
      pcl::io::savePCDFile(config_.mapFile, fullMap_);
      RCLCPP_INFO(this->get_logger(), "Map saved to %s!", config_.mapFile.c_str());
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simple>());
  rclcpp::shutdown();
  return 0;
}
