#ifndef FPFH_EXTRACTOR_H
#define FPFH_EXTRACTOR_H

#include "beginner_tutorials/DesExtractors/DescriptorExtractor.h"
#include <pcl/features/fpfh.h>

namespace enc {

class FPFHExtractor : public DescriptorExtractor
{
private:

public:

  FPFHExtractor();

  virtual std::vector<std::vector<double> > extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

}


#endif