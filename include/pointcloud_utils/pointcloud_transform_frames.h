/**
 * pointcloud_transform_frames.h
 * @author Luca Bartolomei, V4RL
 * @brief  Class to transform a pointcloud in a target frame and publish it 
 * @date   26.09.2019
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

namespace pointcloud_transform_frames {

class PointcloudTransformFrames {

public:
   typedef sensor_msgs::PointCloud2 PointcloudROS;
   typedef pcl::PointCloud<pcl::PointXYZI> PointcloudPCL ;

   /**
   * @brief Class constructor
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   * @param[in] agent_id : ID of current agent
   */
    PointcloudTransformFrames(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

    /**
     * @brief Destructor
     */
    ~PointcloudTransformFrames();

private:
    /**
     * @brief Subscriber for the dense stereo pointcloud for an agent
     * @param[in] pcl_msg : pointcloud in ROS msg format
     */
    void pointCloudCallback(const PointcloudROS::ConstPtr &pcl_msg);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber pcl_sub_;
    ros::Publisher pcl_pub_;

    tf::TransformListener tf_listener_;

    std::string target_frame_;
    std::string pointcloud_topic_input_;
    std::string pointcloud_topic_output_;
}; // end class pointcloud filter

} // end namespace pointcloud_filter
