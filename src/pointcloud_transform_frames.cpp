/**
 * pointcloud_filter.cpp
 * @author Luca Bartolomei, V4RL
 * @date   26.09.2019
 */

#include "pointcloud_utils/pointcloud_transform_frames.h"

#include <tf_conversions/tf_eigen.h>

namespace pointcloud_transform_frames {

  PointcloudTransformFrames::PointcloudTransformFrames(
       const ros::NodeHandle& nh,
       const ros::NodeHandle& nh_private)
       : nh_(nh), nh_private_(nh_private), target_frame_("lidar"),
         pointcloud_topic_input_("pointcloud_in"),
         pointcloud_topic_output_("pointcloud_out") {

    // Get parameters
    std::string ns(
            "pointcloud_transform_frames_node/");
    if(!nh_.getParam(ns + "target_frame", target_frame_)) {
      ROS_WARN("[PCL Transform Frames] Target frame name not specified");
    }

    if(!nh_.getParam(ns + "pointcloud_topic_input", pointcloud_topic_input_)) {
      ROS_WARN("[PCL Transform Frames] Pointcloud input name not specified");
    }

    if(!nh_.getParam(ns + "pointcloud_topic_output", pointcloud_topic_output_)) {
      ROS_WARN("[PCL Transform Frames] Pointcloud output name not specified");
    }

    // set pcl subscriber
    pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            pointcloud_topic_input_, 1,
            &PointcloudTransformFrames::pointCloudCallback, this);

    // set pcl publisher
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            pointcloud_topic_output_, 1);
  }

  PointcloudTransformFrames::~PointcloudTransformFrames() {}

  void PointcloudTransformFrames::pointCloudCallback(
        const PointcloudROS::ConstPtr &pcl_msg) {
    // Get the original pointcloud in the camera frame
    PointcloudPCL::Ptr original_cloud(new PointcloudPCL);
    pcl::fromROSMsg(*pcl_msg, *original_cloud);

    // Transform the pointcloud in the world frame
    ros::Time time_to_lookup = (ros::Time) pcl_msg->header.stamp;

    if (!tf_listener_.canTransform(target_frame_, 
                                   pcl_msg->header.frame_id,
                                   time_to_lookup)) {
      time_to_lookup = ros::Time(0);
      ROS_WARN("Using latest TF transform instead of timestamp match.");
    }

    tf::StampedTransform tf_transform_stamped;
    try {
      tf_listener_.lookupTransform(target_frame_, pcl_msg->header.frame_id,
                                   time_to_lookup, tf_transform_stamped);
    } catch (tf::TransformException& ex) {
      ROS_ERROR_STREAM(
              "Error getting TF transform from sensor data: " << ex.what());

      // In this case, since we do not have a tf, just pass the pointcloud
      // without changing it
      pcl_pub_.publish(pcl_msg);
      return;
    }

    Eigen::Affine3d affine_tranf;
    tf::transformTFToEigen(tf::Transform(tf_transform_stamped.getRotation(),
                                         tf_transform_stamped.getOrigin()),
                           affine_tranf);

    PointcloudPCL::Ptr lidar_cloud(new PointcloudPCL);
    pcl::transformPointCloud(*original_cloud, *lidar_cloud,
            affine_tranf.cast<float>());

    // Publish
    PointcloudROS lidar_cloud_msg;
    pcl::toROSMsg(*lidar_cloud, lidar_cloud_msg);
    lidar_cloud_msg.header = pcl_msg->header;
    lidar_cloud_msg.header.frame_id = target_frame_;
    pcl_pub_.publish(lidar_cloud_msg);
  }
} // end namespace pointcloud_transform_frames
