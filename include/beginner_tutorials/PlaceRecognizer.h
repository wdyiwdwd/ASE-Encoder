#ifndef PLACE_RECOGNIZER_H
#define PLACE_RECOGNIZER_H

#include "beginner_tutorials/common_include.h"
#include "beginner_tutorials/Config.h"
#include "beginner_tutorials/DesExtractors/DescriptorExtractor.h"
#include "beginner_tutorials/DesExtractors/FPFHExtractor.h"
#include "beginner_tutorials/DesEncoders/DescriptorEncoder.h"
#include "beginner_tutorials/DesEncoders/BoWEncoder.h"
#include "beginner_tutorials/DesEncoders/WPUHEncoder.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace enc {

const int ENC_ISE = 0;
const int ENC_BOW = 1;

const int EXT_FPFH = 0;
const int EXT_PFH = 1;

class PlaceRecognizer
{
private:

    PlaceRecognizer() {};
    
    static PlaceRecognizer* pr;

    DescriptorEncoder* encoder;

    DescriptorExtractor* extractor;

    int count;
    std::vector<std::vector<double> > trainset;
    std::vector<std::vector<double> > features;

    std::vector<int> trainset_devision;

    std::vector<std::vector<double> > positions;

    std::ofstream outputFileStream;

    double scoreFactor;

public:

    ~PlaceRecognizer();

    static void initialize(int featureType, int encodingType, std::string filename);
    static void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<double> position);
    static void recognizePlace(int &placeId, double &dfeatureDistist);

    
};

}

#endif