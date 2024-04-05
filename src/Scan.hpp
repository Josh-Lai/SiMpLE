#pragma once

#ifndef SCAN_H
#define SCAN_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <nanoflann.hpp>
#include <set>
#include <tbb/parallel_for.h>

#include "ConfigParser.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "utils.hpp"

/**
 * @brief A generic class for performing operations on a scan.
 *
 */
class Scan {
public:
  // Container to store the point cloud.
  std::vector<Eigen::Vector4d> ptCloud;

  // Container for a nanoflann-friendly point cloud.
  PointCloud<double> pcForKdTree_;

  Scan();

  /**
   * @brief Construct a new Scan object.
   *
   * @param config The algorithm configuration parameters.
   */
  Scan(ConfigParser &config);

  /**
   * @brief Destroy the Scan object.
   *
   */
  ~Scan();

  /**
   * @brief Read a new .bin scan file.
   *
   * @param fileName Name of the file to read.
   */
  void readScan(std::string fileName);
  /**
   * @brief Read a new scan from ROS2
   *
   * @param pcl Point Cloud From ROS2
   */
  void readScan(sensor_msgs::msg::PointCloud2 pcl);

protected:
  // Point cloud subsample radius.
  double subsampleRadius_;

  // Maximum range of the points in the scan.
  double maxSensorRange_;

  // Minimum range of the points in the scan.
  double minSensorRange_;

  // A container used for radially subsampling the points.
  std::set<int> allPoints_;

  /**
   * @brief Radially subsample the point cloud.
   *
   * @param pts               The points to subsample.
   * @param subsampleRadius   The subsample radius in meters.
   */
  void subsample_(std::vector<Eigen::Vector4d> &pts, double subsampleRadius);

  /**
   * @brief Convert the points to a nanoflann-friendly container.
   *
   * @param pts The points to convert to a nanoflann-friendly container.
   */
  void convertToPointCloudKdTree_(std::vector<Eigen::Vector4d> &pts);

private:
  // Boolean to apply the correcttion factor to the KITTI scans.
  bool kitti_;

  /**
   * @brief Apply the scan correction factor if required and subsample
   *        the scan.
   *
   */
  void processPointCloud_();

  /**
   * @brief Apply a correcttion factor to fix the KITTI scans.
   *
   */
  void correctKittiScan_();
};

#endif
