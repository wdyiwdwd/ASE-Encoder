#include "beginner_tutorials/DesEncoders/WPUHEncoder.h"

namespace enc {

  WPUHEncoder::WPUHEncoder() {
    this->root =  std::shared_ptr<ETNode>(new ETNode());
    this->levelFirst.push_back(this->root);
    this->NTlevel = 0;
  }

  WPUHEncoder::~WPUHEncoder() {}

  void WPUHEncoder::nomalizeOnTraining(std::vector<std::vector<double> >& features) {
    if (features.size() == 0) {
      std::cout << "WARNING: THE SIZE OF FEATURES IS 0 IN NOMALIZATION !!" << std::endl;
      return;
    }
    this->maximums = std::vector<double>(features[0].size(), -10e6);
    this->minimums = std::vector<double>(features[0].size(), 10e6);
    for (int i = 0; i < features.size(); i++) {
      for (int j = 0; j < features[i].size(); j++) {
        if (features[i][j] > this->maximums[j]) {
          maximums[j] = features[i][j];
        }
        if (features[i][j] < this->minimums[j]) {
          minimums[j] = features[i][j];
        }
      }
    }
    for (int i = 0; i < features.size(); i++) {
      for (int j = 0; j < features[i].size(); j++) {
        // if (features[i][j] > this->maximums[j]) {
        //   features[i][j] = 1;
        // }
        // else if (features[i][j] < this->minimums[j]) {
        //   features[i][j] = 0;
        // }
        // else {
        //   features[i][j] = (features[i][j] - minimums[j]) / (maximums[j] - minimums[j]);
        // }
        features[i][j] = (features[i][j] - minimums[j]) / (maximums[j] - minimums[j]);
      }
    }
  }

  void WPUHEncoder::nomalizeOnEncoding(std::vector<std::vector<double> >& features) {
    for (int i = 0; i < features.size(); i++) {
      for (int j = 0; j < features[i].size(); j++) {
        if (features[i][j] > this->maximums[j]) {
          features[i][j] = 1;
        }
        else if (features[i][j] < this->minimums[j]) {
          features[i][j] = 0;
        }
        else {
          features[i][j] = (features[i][j] - minimums[j]) / (maximums[j] - minimums[j]);
        }
      }
    }
  }

  void WPUHEncoder::principalComponentAnalysis(std::vector<std::vector<double> >& features, cv::Mat& PCAMat) {
    if (features.size() == 0) {
      std::cout << "WARNING: THE SIZE OF FEATURES IS 0 IN PCA !!" << std::endl;
      return;
    }
    cv::Mat featuresMat = cv::Mat::zeros(features.size(), features[0].size(), CV_32F);
    for (int i = 0; i < features.size(); i++) {
      float* ptr = featuresMat.ptr<float>(i);
      for (int j = 0; j < features[i].size(); j++) {
        ptr[j] = (float)features[i][j];
      }
    }

    // 去中心化
    cv::Mat means;
    for (int i = 0; i < featuresMat.cols; i++) {
      cv::Mat col = featuresMat.colRange(i, i + 1);
      cv::Mat mean, std;
      cv::meanStdDev(col, mean, std);
      means.push_back(mean);
    }
    means = means.t();
    means.convertTo(means, CV_32F);
    cv::Mat decentralizedMat;
    for(int i = 0; i < featuresMat.rows; i++) {
      decentralizedMat.push_back(featuresMat.rowRange(i, i + 1) - means);
    }

    cv::Mat S,U;
    cv::SVD::compute(decentralizedMat, S, U, V, cv::SVD::FULL_UV);
    std::cout << "decentralizedMat: " << decentralizedMat.rows << " " << decentralizedMat.cols << std::endl;
    std::cout << "S: " << S.rows << " " << S.cols << std::endl;
    std::cout << "U: " << U.rows << " " << U.cols << std::endl;
    std::cout << "V: " << V.rows << " " << V.cols << std::endl;
    std::cout << S << std::endl;
    PCAMat = decentralizedMat * V.t();
    std::cout << "PCAMat: " << PCAMat.rows << " " << PCAMat.cols << std::endl;
  }

  std::vector<double> WPUHEncoder::computeHyperplane(std::vector<double> &data1D, int numOfPlane) {
    std::vector<double> hyperplanes;
    for (int i = 0; i < numOfPlane; i++) {
      hyperplanes.push_back(data1D[i * (data1D.size() - 1) / numOfPlane]);
    }
    return hyperplanes;
  }

  void WPUHEncoder::hyperplaneDivision(cv::Mat& PCAMat) {
    for (int i = 15; i < 16; i++) {
    // for (int i = 0; i < PCAMat.cols; i++) {
      std::vector<double> col;
      for (int j = 0; j < PCAMat.rows; j++) {
        col.push_back(PCAMat.at<float>(j, i));
      }
      std::sort(col.begin(), col.end());
      std::ofstream of("./points2.txt");
      for (int j = 0; j < col.size(); j++) {
        std::cout << col[j] << " ";
        of << col[j] << " ";
      }
      std::cout << std::endl;
      of << std::endl;
      of.close();
    }


    const int DEMENTION = 5;
    for (int i = 0; i < DEMENTION; i++) {
      std::vector<double> col;
      for (int j = 0; j < PCAMat.rows; j++) {
        col.push_back(PCAMat.at<float>(j, i));
      }
      std::sort(col.begin(), col.end());
      int numOfPlane = 3 + (rand() % 3); 
      std::vector<double> hyperplanes = this->computeHyperplane(col, numOfPlane);

      std::shared_ptr<ETNode> currentNode = levelFirst[NTlevel];
      while (currentNode.get() != NULL) {
        for (int k = 0; k < numOfPlane; k++) {
          std::shared_ptr<ETNode> tempNT(new ETNode());
          tempNT->hyperplane = hyperplanes[k];
          tempNT->weight = 0;
          currentNode->children.push_back(tempNT);
        }
        currentNode = currentNode->next;
      } 

      currentNode = levelFirst[NTlevel];
      std::shared_ptr<ETNode> nextNode = currentNode->next;
      while (nextNode.get() != NULL) {
        for (int k = 0; k < currentNode->children.size() - 1; k++) {
          currentNode->children[k]->next = currentNode->children[k + 1];
        }
        currentNode->children[currentNode->children.size() - 1]->next = nextNode->children[0];
        currentNode = nextNode;
        nextNode = nextNode->next;
      }
      for (int k = 0; k < currentNode->children.size() - 1; k++) {
        currentNode->children[k]->next = currentNode->children[k + 1];
      }

      
    }
  }

  void WPUHEncoder::setParametersFromDefaultConfig() {
    // ...
  }

  void WPUHEncoder::train(std::vector<std::vector<double> >& features) {
    this->nomalizeOnTraining(features);
    // for (int i = 0; i < 5; i++) {
    //   for (int j = 0; j < features[i].size(); j++) {
    //     std::cout << features[i][j] << " ";
    //   }
    //   std::cout << std::endl;
    // }
    // std::cout << std::endl; 
    cv::Mat PCAMat;
    this->principalComponentAnalysis(features, PCAMat);
    this->hyperplaneDivision(PCAMat);
  }

  std::vector<double> WPUHEncoder::encode(std::vector<std::vector<double> >& features) const{
    return std::vector<double>({1, 1, 1});
  }

  

}