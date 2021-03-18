#ifndef LOCAL_DESCRIPTOR_ENCODER_H
#define LOCAL_DESCRIPTOR_ENCODER_H

#include "beginner_tutorials/common_include.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace enc {

class LocalDescriptorEncoder
{
protected:

public:
    LocalDescriptorEncoder() {};

    virtual void setParametersFromDefaultConfig() = 0;

    virtual void train(std::vector<std::vector<double> >& features) = 0;

    virtual std::vector<double> encode(std::vector<std::vector<double> >& features) const = 0;
    
};

}

#endif