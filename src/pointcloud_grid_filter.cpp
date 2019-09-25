/**
 * @author Luca Bartolomei, V4RL
 * @date   25.09.2019
 */

#include "pointcloud_utils/pointcloud_grid_filter.h"

#include <pcl/filters/voxel_grid.h>

namespace pointcloud_grid_filter {

PointcloudGridFilter::PointcloudGridFilter(
        const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh), nh_private_(nh_private), input_topic_("pointcloud_in"),
          output_topic_("pointcloud_out"), throttle_time_(0.1),
          filter_size_(0.05) {

  // Read parameters
  if (!nh_private_.getParam("input_topic", input_topic_)) {
    ROS_WARN_STREAM(
            "[Pointcloud Grid Filter] Input topic not specified. Using "
            "'pointcloud'");
  }

  if (!nh_private_.getParam("output_topic", output_topic_)) {
    ROS_WARN_STREAM(
            "[Pointcloud Grid Filter] Output topic not specified. Using 'mesh'");
  }

  if (!nh_private_.getParam("throttle_time", throttle_time_)) {
    ROS_WARN_STREAM(
            "[Pointcloud Grid Filter] Throttle time not specified. Using 0.1 s");
  }

  if (!nh_private_.getParam("filter_size", filter_size_)) {
    ROS_WARN_STREAM(
            "[Pointcloud Grid Filter] Filter size not specified. Using 0.05 m");
  }

  // Initialize ROS
  pcl_sub_ = nh_private_.subscribe(input_topic_, 10,
          &PointcloudGridFilter::pointCloudCallback, this);
  filter_pcl_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>(
          output_topic_, 1000, true);

  // Initialize time for throttling
  last_pointcloud_time_ = ros::Time::now().toSec();
}

PointcloudGridFilter::~PointcloudGridFilter() {}

void PointcloudGridFilter::pointCloudCallback(
        const PointcloudROS::ConstPtr &pcl_msg) {
  if(filter_pcl_pub_.getNumSubscribers() == 0 ||
     (pcl_msg->header.stamp.toSec() - last_pointcloud_time_) < throttle_time_) {
    last_pointcloud_time_ = pcl_msg->header.stamp.toSec();
    return;
  }

  // Get the ROS message in PCL format
  pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr pcl_cloud_filtered (new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(*pcl_msg, *pcl_cloud);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  voxel_filter.setInputCloud (pcl_cloud);
  voxel_filter.setLeafSize (filter_size_, filter_size_, filter_size_);
  voxel_filter.filter (*pcl_cloud_filtered);

  // Publish filtered pointcloud
  last_pointcloud_time_ = pcl_msg->header.stamp.toSec();
  sensor_msgs::PointCloud2 pcl_filtered_msg;
  pcl_conversions::fromPCL(*pcl_cloud_filtered, pcl_filtered_msg);

  filter_pcl_pub_.publish(pcl_filtered_msg);
}

}

