#ifndef PLACE_RECOGNIZER_H
#define PLACE_RECOGNIZER_H

#include "beginner_tutorials/common_include.h"
#include "beginner_tutorials/Config.h"
#include "beginner_tutorials/LocalDescriptorExtractor.h"
#include "beginner_tutorials/LocalDescriptorEncoder.h"
#include "beginner_tutorials/BoWEncoder.h"
#include "beginner_tutorials/WPUHEncoder.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace enc {

const int ENC_ISE = 0;
const int ENC_BOW = 1;

class PlaceRecognizer
{
private:

    PlaceRecognizer() {};
    
    static PlaceRecognizer* pr;

    LocalDescriptorEncoder* encoder;

    int count;
    std::vector<std::vector<double> > trainset;
    std::vector<std::vector<double> > features;

    std::vector<int> trainset_devision;

    std::vector<std::vector<double> > positions;

    std::ofstream outputFileStream;

    double scoreFactor;

public:

    ~PlaceRecognizer();

    static void initialize(int encodingType, std::string filename);
    static void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<double> position);
    static void recognizePlace(int &placeId, double &dfeatureDistist);

    
};

}

#endif