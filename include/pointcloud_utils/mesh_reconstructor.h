/**
 * pointcloud_filter.h
 * @author Luca Bartolomei, V4RL
 * @brief  Class to that gets an unordered set of points and creates a mesh
 *         out of it
 * @date   19.09.2019
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

namespace mesh_reconstructor {

class MeshReconstructor {

public:
   typedef sensor_msgs::PointCloud2 PointcloudROS;
   typedef pcl::PointCloud<pcl::PointXYZ> PointcloudPCL ;

   /**
   * @brief Class constructor
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   */
    MeshReconstructor(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);

    /**
     * @brief Destructor
     */
    ~MeshReconstructor();

private:
    /**
     * @brief Subscriber for the input pointcloud
     * @param[in] pcl_msg : pointcloud in ROS msg format
     */
    void pointCloudCallback(const PointcloudROS::ConstPtr &pcl_msg);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber pcl_sub_;
    ros::Publisher mesh_pub_;

    std::string input_topic_;
    std::string output_topic_;
}; // end class mesh_reconstructor

} // end namespace mesh_reconstructor
