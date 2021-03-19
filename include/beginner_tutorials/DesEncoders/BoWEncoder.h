#ifndef BOW_ENCODER_H
#define BOW_ENCODER_H

#include "beginner_tutorials/common_include.h"
#include "beginner_tutorials/DesEncoders/DescriptorEncoder.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/ml/ml.hpp>

namespace enc {

class BoWEncoder : public DescriptorEncoder
{
private:

    int K;
    cv::Mat centers;
    cv::TermCriteria criteria;
    int initType;
    int attempts;

protected:

public:

    BoWEncoder();
    ~BoWEncoder();

    virtual void setParametersFromDefaultConfig();

    virtual void train(std::vector<std::vector<double> >& features);

    virtual std::vector<double> encode(std::vector<std::vector<double> >& features) const;
    
};

}

#endif