/**
 * pointcloud_grid_filter.h
 * @author Luca Bartolomei, V4RL
 * @brief  Class to filter a pointcloud with a grid filter
 * @date   25.09.2019
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pointcloud_grid_filter {

class PointcloudGridFilter {

public:
   typedef sensor_msgs::PointCloud2 PointcloudROS;
   typedef pcl::PointCloud<pcl::PointXYZ> PointcloudPCL ;

   /**
   * @brief Class constructor
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   */
    PointcloudGridFilter(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);

    /**
     * @brief Destructor
     */
    ~PointcloudGridFilter();

private:
    /**
     * @brief Subscriber for input pointcloud
     * @param[in] pcl_msg : pointcloud in ROS msg format
     */
    void pointCloudCallback(const PointcloudROS::ConstPtr &pcl_msg);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber pcl_sub_;
    ros::Publisher filter_pcl_pub_;

    std::string input_topic_;
    std::string output_topic_;

    double filter_size_;
    double throttle_time_;
    double last_pointcloud_time_;
}; // end class mesh_reconstructor

} // end namespace mesh_reconstructor
