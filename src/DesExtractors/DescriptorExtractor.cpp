#include "beginner_tutorials/DesExtractors/DescriptorExtractor.h"

namespace enc {

DescriptorExtractor::DescriptorExtractor() {}

void DescriptorExtractor::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, float voxel_x, float voxel_y, float voxel_z) {
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloudIn);
  sor.setLeafSize (voxel_x, voxel_y, voxel_z);
  sor.filter (*cloudOut);
  // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  // approximate_voxel_filter.setLeafSize (voxel_x, voxel_y, voxel_z);
  // approximate_voxel_filter.setInputCloud (cloudIn);
  // approximate_voxel_filter.filter (*cloudOut);
  std::cout << "Filtered cloud contains " << cloudOut->size()
          << " data points from input_pcd" << std::endl;
}

void DescriptorExtractor::normalEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int K) {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  // Use all neighbors in a sphere of radius 3cm
  ne.setKSearch(K);
  ne.compute(*normals);
  std::cout << "Nomals contains " << normals->size()
          << " and points from input cloud contains " << cloud->size() << std::endl;
  // cloud_normals->size () should have the same size as the input cloud->size ()*
}

}