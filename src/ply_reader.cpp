/**
 * @author Luca Bartolomei, V4RL
 * @date   12.09.2019
 */

#include "pointcloud_utils/ply_reader.h"

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

PlyReader::PlyReader(
    const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), output_frame_("world"),
      output_topic_("pointcloud"), has_ply_file_(false),
      has_collada_file_(false), scale_collada_(1.0), position_collada_x_(0.0),
      position_collada_y_(0.0), position_collada_z_(0.0),
      rotation_collada_roll_(0.0), rotation_collada_pitch_(0.0),
      rotation_collada_yaw_(0.0) {

  // Read parameters
  if(!nh_private_.getParam("output_frame", output_frame_)) {
    ROS_WARN("[PLY Reader] Output frame not specified. Using 'world'");
  }

  if(!nh_private_.getParam("output_topic", output_topic_)) {
    ROS_WARN("[PLY Reader] Output topic not specified. Using 'pointcloud'");
  }

  if(!nh_private_.getParam("ply_path", ply_path_)) {
    ROS_WARN("[PLY Reader] PLY File to process not specified! Close the "
                    "process");
  } else {
    has_ply_file_ = true;
  }

  if(!nh_private_.getParam("collada_path", collada_path_)) {
    ROS_WARN("[PLY Reader] Collada File to process not specified! Close "
                    "the process");
  } else {
    has_collada_file_ = true;
  }

  if(!nh_private_.getParam("scale_collada", scale_collada_)) {
    ROS_WARN("[PLY Reader] Collada scale not specified. Using 1.0");
  }

  if(!nh_private_.getParam("position_collada_x", position_collada_x_)) {
    ROS_WARN("[PLY Reader] Collada X position not specified. Using 0.0");
  }

  if(!nh_private_.getParam("position_collada_y", position_collada_y_)) {
    ROS_WARN("[PLY Reader] Collada Y position not specified. Using 0.0");
  }

  if(!nh_private_.getParam("position_collada_z", position_collada_z_)) {
    ROS_WARN("[PLY Reader] Collada Z position not specified. Using 0.0");
  }

  if(!nh_private_.getParam("rotation_collada_roll", rotation_collada_roll_)) {
    ROS_WARN("[PLY Reader] Collada roll rotation not specified. Using 0.0");
  }

  if(!nh_private_.getParam("rotation_collada_pitch", rotation_collada_pitch_)) {
    ROS_WARN("[PLY Reader] Collada pitch rotation not specified. Using 0.0");
  }

  if(!nh_private_.getParam("rotation_collada_yaw", rotation_collada_yaw_)) {
    ROS_WARN("[PLY Reader] Collada yaw rotation not specified. Using 0.0");
  }

  // Initialize ROS
  pointcloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>(
          output_topic_, 1000, true);
  collada_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
          "collada", 1000, true);

  read_file_srv_ = nh_private_.advertiseService(
          "read_file", &PlyReader::readFileServiceCallback, this);
  publish_collada_srv_= nh_private_.advertiseService(
          "publish_collada", &PlyReader::publishColladaServiceCallback, this);
}

PlyReader::~PlyReader() {}

bool PlyReader::readFileServiceCallback(
        std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  ROS_INFO("[PLY Reader] Start processing PLY file...");
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ply_cloud (
          new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (pcl::io::loadPLYFile<pcl::PointXYZRGBA> (ply_path_, *ply_cloud) == -1) {
    ROS_ERROR("[PLY Reader] Could not read file");
    res.success = false;
    return false;
  } else {
    ROS_INFO("[PLY Reader] Successfully read the ply file");
  }

  // Transform the pointcloud in ros message
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*ply_cloud, ros_cloud);
  ros_cloud.header.stamp = ros::Time::now();
  ros_cloud.header.frame_id = output_frame_;

  pointcloud_pub_.publish(ros_cloud);
  ROS_INFO("[PLY Reader] Published pointcloud to ROS");

  res.success = true;
  return true;
}

bool PlyReader::publishColladaServiceCallback(
          std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.mesh_resource = "file://" + collada_path_;
  marker.mesh_use_embedded_materials = true;
  marker.scale.x = marker.scale.y = marker.scale.z = scale_collada_;

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = output_frame_;
  marker.lifetime = ros::Duration();

  marker.pose.position.x = position_collada_x_;
  marker.pose.position.y = position_collada_y_;
  marker.pose.position.z = position_collada_z_;
  marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
     rotation_collada_roll_, rotation_collada_pitch_, rotation_collada_yaw_);
  marker.color.a = 1.0;

  ROS_INFO("[PLY Reader] Published mesh");
  collada_pub_.publish(marker);

  res.success = true;
  return true;
}


