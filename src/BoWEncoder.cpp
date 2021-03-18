#include "beginner_tutorials/BoWEncoder.h"

namespace enc {

  BoWEncoder::BoWEncoder() {

  }

  BoWEncoder::~BoWEncoder() {}

  void BoWEncoder::setParametersFromDefaultConfig() {
    this->K = 200;
    this->criteria = cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100000, 0.000001);
    this->attempts = 10;
    this->initType = cv::KMEANS_PP_CENTERS;
  }

  void BoWEncoder::train(std::vector<std::vector<double> >& features) {
    std::cout << "BoW trainning begin" << std::endl;
    cv::Mat dataMat = cv::Mat::zeros(features.size(), features[0].size(), CV_32F);
    for (int i = 0; i < features.size(); i++) {
      float* ptr = dataMat.ptr<float>(i);
      for (int j = 0; j < features[i].size(); j++) {
        ptr[j] = (float)features[i][j];
      }
    }
    this->centers = cv::Mat::zeros(this->K, dataMat.cols, dataMat.type());
    cv::Mat responsesMat = cv::Mat::zeros(dataMat.rows, 1, CV_8U);
    cv::kmeans(dataMat, this->K, responsesMat, this->criteria, this->attempts, this->initType, this->centers);
    std::cout << "(" << centers.rows << ", " << centers.cols << ")" << std::endl;
    std::cout << "BoW trainning end" << std::endl;
  }

  std::vector<double> BoWEncoder::encode(std::vector<std::vector<double> >& features) const{
    std::vector<double> feature(centers.rows, 0);
    for (int i = 0; i < features.size(); i++) {
      cv::Mat dataMat = cv::Mat::zeros(1, features[i].size(), CV_32F);
      float* ptr = dataMat.ptr<float>(0);
      for (int j = 0; j < features[i].size(); j++) {
        ptr[j] = (float)features[i][j];
      }
      int nearest = 0;
      double mindist = 10e6;
      // std::cout << "dataMat: " << dataMat << std::endl;
      // std::cout << "centers: " << centers.rowRange(0, 1) << std::endl;
      // std::cout << "dist1: " << cv::norm(dataMat - centers.rowRange(0, 1), CV_L2);
      // std::cout << "dist2: " << cv::norm(dataMat, centers.rowRange(0, 1), CV_L2);
      for (int j = 0; j < centers.rows; j++) {
        double dist = cv::norm(dataMat, centers.rowRange(j, j + 1), CV_L2);
        if (dist < mindist) {
          mindist = dist;
          nearest = j;
        }
      }
      feature[nearest] += 1.0 / features.size(); 
    }
    return feature;
  }

  

}