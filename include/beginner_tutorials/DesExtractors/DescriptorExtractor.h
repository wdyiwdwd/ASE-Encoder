#ifndef DESCRIPTOR_EXTRACTOR_H
#define DESCRIPTOR_EXTRACTOR_H

#include "beginner_tutorials/common_include.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace enc {

class DescriptorExtractor
{
private:

public:
    DescriptorExtractor();

    virtual std::vector<std::vector<double> > extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) = 0;

    static void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, float voxel_x, float voxel_y, float voxel_z);

    static void normalEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int K);
    
};

}

#endif