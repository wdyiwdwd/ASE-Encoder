#ifndef WPUH_ENCODER_H
#define WPUH_ENCODER_H

#include "beginner_tutorials/common_include.h"
#include "beginner_tutorials/LocalDescriptorEncoder.h"
#include "beginner_tutorials/Config.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/ml/ml.hpp>


namespace enc {

struct ETNode {
    double hyperplane;
    double weight;
    std::shared_ptr<ETNode> next;
    std::vector<std::shared_ptr<ETNode> > children;
};


class WPUHEncoder : public LocalDescriptorEncoder
{
private:
    std::vector<double> maximums;
    std::vector<double> minimums;
    void nomalizeOnTraining(std::vector<std::vector<double> >& features);
    void nomalizeOnEncoding(std::vector<std::vector<double> >& features);

    // std::vector<std::vector<double> > directions;
    // std::vector<std::vector<double> > hyperplanes;

    cv::Mat V;

    void principalComponentAnalysis(std::vector<std::vector<double> >& features, cv::Mat& PCAMat);

    std::vector<double> computeHyperplane(std::vector<double> &data1D, int numOfPlane);
    void hyperplaneDivision(cv::Mat& PCAMat);

    void encodingTreeConstruction(std::vector<std::vector<double> >& features);

    std::shared_ptr<ETNode> root;
    std::vector<std::shared_ptr<ETNode> > levelFirst;
    int NTlevel;

protected:


public:

    WPUHEncoder();
    ~WPUHEncoder();

    virtual void setParametersFromDefaultConfig();

    virtual void train(std::vector<std::vector<double> >& features);

    virtual std::vector<double> encode(std::vector<std::vector<double> >& features) const;
    
};

}

#endif