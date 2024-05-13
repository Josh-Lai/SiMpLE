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

        // The custom frame to output the estimated frame translation
        // If this parameter is empty, the Point Cloud frame will be used
        std::string outputFrame;

        // If enabled, will store the parsed point clouds in RAM and save the map to a file
        bool saveToFile;

        // File path to save the file to
        std::string mapFile;

        // Downsampling the saved file. If this parameter is zero, downsampling will not employed
        double fileDownsample;

    private:
        // Path to the algorithm configuration file.
        std::string yamlFilePath_;
};
