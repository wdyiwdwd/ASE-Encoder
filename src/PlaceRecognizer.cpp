#include "beginner_tutorials/PlaceRecognizer.h"

namespace enc {

  PlaceRecognizer::~PlaceRecognizer() {
    pr->outputFileStream.close();
    delete pr->encoder;
    delete pr;
  }

  void PlaceRecognizer::initialize(int encodingType, std::string filename) {
    if (pr != nullptr) return;

    pr = new PlaceRecognizer();

    LocalDescriptorExtractor::initialize();

    switch (encodingType)
    {
    case ENC_ISE:
      pr->encoder = new WPUHEncoder();
      break;

    case ENC_BOW:
      pr->encoder = new BoWEncoder();
      break;
    }

    pr->outputFileStream.open(filename);

    pr->scoreFactor = -1;

    pr->count = 0;
    pr->encoder->setParametersFromDefaultConfig();
  }

  void PlaceRecognizer::addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<double> position) {

    const int trainnum = enc::Config::get<int>("trainset_num");
    pr->trainset_devision.push_back(0);

    std::vector<std::vector<double> > localfeatures = LocalDescriptorExtractor::extract(cloud);
    
    if (pr->count < trainnum) {
      for (int i = 0; i < localfeatures.size(); i++) {
        pr->trainset.push_back(localfeatures[i]);
      }
      pr->trainset_devision.push_back(localfeatures.size());
    }
    else if (pr->count == trainnum) {
      for (int i = 0; i < localfeatures.size(); i++) {
        pr->trainset.push_back(localfeatures[i]);
      }
      pr->trainset_devision.push_back(localfeatures.size());
      pr->encoder->train(pr->trainset);
      for (int i = 0; i < pr->trainset_devision.size() - 1; i++) {
        std::vector<std::vector<double> > trainset_localfeatures;
        for (int j = 0; j < pr->trainset_devision[i + 1]; i++) {
          trainset_localfeatures.push_back(pr->trainset[j + pr->trainset_devision[i]]);
        }
        std::vector<double> f = pr->encoder->encode(localfeatures);
        pr->features.push_back(f);
      }
    }
    else {
      std::vector<double> f = pr->encoder->encode(localfeatures);
      pr->features.push_back(f);
      for (int i = 0; i < f.size(); i++) {
        std::cout << f[i] << " ";
      }
      std::cout << std::endl;
    }

    pr->positions.push_back(position);
    
    pr->count++;

    std::cout << "count: " << pr->count << std::endl;
    std::cout << "train numbers: " << pr->trainset.size() << std::endl;
    std::cout << "feature numbers: " << pr->features.size() << std::endl;
  }

  void PlaceRecognizer::recognizePlace(int& placeId, double& featureDist) {
    const int nearestNum = enc::Config::get<int>("trainset_num");
    int searchScope = (pr->features.size() - 1 > nearestNum)? pr->features.size() - nearestNum - 1 : 0;
    int mostSimilarId = -1;
    double nearestFeatureDist = 10e6;
    int nearestId = -1;
    double nearestDist = 10e6;
    std::vector<double> groundTruthDistances;
    for (int i = 0; i < searchScope; i++) {
      std::vector<double> nfeature = pr->features[pr->features.size() - 1];
      std::vector<double> mfeature = pr->features[i];
      double featureDist = 0;
      for (int j = 0; j < nfeature.size(); j++) {
        featureDist += (nfeature[j] - mfeature[j]) * (nfeature[j] - mfeature[j]);
      }
      featureDist = sqrt(featureDist);
      if (featureDist < nearestFeatureDist) {
        mostSimilarId = i;
        nearestFeatureDist = featureDist;
      }

      double currentX = pr->positions[pr->positions.size() - 1][0];
      double currentY = pr->positions[pr->positions.size() - 1][1];
      double currentZ = pr->positions[pr->positions.size() - 1][2];
      double histricalX = pr->positions[i][0];
      double histricalY = pr->positions[i][1];
      double histricalZ = pr->positions[i][2];
      double spaceDist = sqrt((currentX - histricalX) * (currentX - histricalX) + (currentY - histricalY) * (currentY - histricalY) + (currentZ - histricalZ) * (currentZ - histricalZ));
      groundTruthDistances.push_back(spaceDist);
      if (spaceDist < nearestDist) {
        nearestDist = spaceDist;
        nearestId = i;
      }
    }

    if (searchScope > 0) {
      double mostSimilarSpaceDist = mostSimilarId == -1 ? 10e6 : groundTruthDistances[mostSimilarId];

      if (pr->scoreFactor == -1) 
        pr->scoreFactor = nearestFeatureDist;
      double score = (1 - exp(- pr->scoreFactor / nearestFeatureDist)) / (1 + exp(- pr->scoreFactor / nearestFeatureDist));

      pr->outputFileStream << score << " " << mostSimilarSpaceDist << " " << nearestDist << " " << mostSimilarId << " "  << " " << nearestId << std::endl; 

      std::cout << score << " " << mostSimilarSpaceDist << " " << nearestDist << " " << mostSimilarId << " "  << " " << nearestId << std::endl; 
    }
    
    placeId = mostSimilarId;
    featureDist = nearestFeatureDist;
  }

  PlaceRecognizer* PlaceRecognizer::pr = nullptr;
}