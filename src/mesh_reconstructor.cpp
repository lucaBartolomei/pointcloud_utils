/**
 * @author Luca Bartolomei, V4RL
 * @date   19.09.2019
 */

#include "pointcloud_utils/mesh_reconstructor.h"

#include <chrono>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

namespace mesh_reconstructor {

MeshReconstructor::MeshReconstructor(
        const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh), nh_private_(nh_private), input_topic_("pointcloud"),
          output_topic_("mesh") {

  // Read parameters
  if (!nh_private_.getParam("input_topic", input_topic_)) {
    ROS_WARN_STREAM(
            "[Mesh Reconstructor] Input topic not specified. Using "
            "'pointcloud'");
  }

  if (!nh_private_.getParam("output_topic", output_topic_)) {
    ROS_WARN_STREAM(
            "[Mesh Reconstructor] Output topic not specified. Using 'mesh'");
  }

  // Initialize ROS
  pcl_sub_ = nh_private_.subscribe(input_topic_, 10,
          &MeshReconstructor::pointCloudCallback, this);
  mesh_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>(
          output_topic_, 1000, true);
}

MeshReconstructor::~MeshReconstructor() {}

void MeshReconstructor::pointCloudCallback(
        const PointcloudROS::ConstPtr &pcl_msg) {

  // This procedure is copied from PCL tutorial:
  // http://pointclouds.org/documentation/tutorials/greedy_projection.php#greedy-triangulation
  auto start = std::chrono::high_resolution_clock::now();

  // Get the pointcloud
  PointcloudPCL::Ptr pcl_cloud(new PointcloudPCL);
  pcl::fromROSMsg(*pcl_msg, *pcl_cloud);

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (
          new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pcl_cloud);
  normal_estimation.setInputCloud (pcl_cloud);
  normal_estimation.setSearchMethod (tree);
  normal_estimation.setKSearch (20);
  normal_estimation.compute (*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (
          new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*pcl_cloud, *normals, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (
          new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.01);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/36); // 5 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Publish a ROS message
  PointcloudROS mesh_msg;
  PointcloudPCL::Ptr mesh_pcl(new PointcloudPCL);

  pcl::fromPCLPointCloud2(triangles.cloud, *mesh_pcl);
  pcl::toROSMsg(*mesh_pcl, mesh_msg);
  mesh_pub_.publish(mesh_msg);

  // Timings
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  ROS_INFO_STREAM("[Mesh Reconstructor] Mesh construction took " <<
                  elapsed.count() << " s");
}

}

