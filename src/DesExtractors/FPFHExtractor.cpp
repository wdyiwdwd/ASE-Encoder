#include "beginner_tutorials/DesExtractors/FPFHExtractor.h"

namespace enc {

FPFHExtractor::FPFHExtractor() {}

std::vector<std::vector<double> > FPFHExtractor::extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  DescriptorExtractor::downsample(cloud, downsampling_cloud, 0.3f, 0.3f, 0.3f);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  DescriptorExtractor::normalEstimate(downsampling_cloud, normals, 10);
    // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (downsampling_cloud);
  fpfh.setInputNormals (normals);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  fpfh.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (0.5);
  // Compute the features
  fpfh.compute (*fpfhs);

  cv::Mat featureMat = cv::Mat::zeros(fpfhs->size(), 33, CV_32F);
  for (int i = 0; i < fpfhs->size(); i++) {
      float* ptr = featureMat.ptr<float>(i);
      for (int j = 0; j < 33; j++) {
        ptr[j] = fpfhs->points[i].histogram[j];
      }
  }

  cv::Mat means;
  for (int i = 0; i < featureMat.cols; i++) {
    cv::Mat col = featureMat.colRange(i, i + 1);
    cv::Mat mean, std;
    cv::meanStdDev(col, mean, std);
    means.push_back(mean);
  }
  means = means.t();
  means.convertTo(means, CV_32F);


  cv::Mat dists = cv::Mat::zeros(1, fpfhs->size(), CV_32F);
  float* dists_ptr = dists.ptr<float>(0); 
  for (int i = 0; i < fpfhs->size(); i++) {
    cv::Mat f = featureMat.rowRange(i, i + 1).clone();
    dists_ptr[i] = (float)cv::norm(f - means, CV_L2);
  }

  // std::cout << "dists: " << dists.rows << " " << dists.cols << std::endl;

  cv::Mat dist_mean, dist_std;
  cv::meanStdDev(dists, dist_mean, dist_std);

  // std::cout << "dist_mean: " << dist_mean << std::endl;
  // std::cout << "dist_std: " << dist_std << std::endl;

  double d_mean = dist_mean.at<double>(0, 0);
  double d_std = dist_std.at<double>(0, 0);

  // std::cout << "mean: " << d_mean << "   " << "stds: " << d_std << std::endl;

  std::vector<std::vector<double> > features;
  for (int i = 0; i < fpfhs->size(); i++) {
      if (dists.at<float>(0, i) >= d_mean + 2 * d_std || dists.at<float>(0, i) <= d_mean - 2 * d_std) {
        std::vector<double> rowfeature;
        for (int j = 0; j < 33; j++) {
          rowfeature.push_back(fpfhs->points[i].histogram[j]);
        }
        features.push_back(rowfeature);
        // std::cout << "feature: " << std::endl;
        // for (int k = 0; k < rowfeature.size(); k++) {
        //   std::cout << rowfeature[k] << " ";
        // }
        // std::cout << std::endl;
        }
  }
  std::cout << "features: " << features.size() << std::endl;
  return features;

  // return std::vector<std::vector<double> >();
}

}