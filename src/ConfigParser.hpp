#pragma once

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

/**
 * @brief A class to handle the algorithm configuration parsing.
 * Reconfigured to store the results from the ROS2 parameters
 * 
 */
class ConfigParser
{
    public:
        // Print the algorithm configuration parameters.
        bool verbose;

        // Correct the KITTI scans.
        bool kitti;

        // The standard deviation used to calculate proximity-based reward.
        double sigma;

        // The spatial separation used to subsample the existing map.
        double rMap;

        // The spatial separation used to subsample new point cloud data.
        double rNew;

        // The optimization solver’s exit condition is described as a minimum
        // reward improvement between iterations.
        double convergenceTol;

        // The sensor's maximum range.
        double maxSensorRange;
        
        // The sensor's minimum range.
        double minSensorRange;

        // Use live point clouds
        bool useLivePointCloud;
        
        // Must declare these parameters if a live point cloud is used
        // Topic that Point Cloud is published to 
        std::string pointCloudTopic;

        // The frame that the map is located in
        std::string mapFrame;

        // Path to the scan files.
        std::string scanPath;

        // Name of the output file.
        std::string outputFileName;

    private:
        // Path to the algorithm configuration file.
        std::string yamlFilePath_;
};
