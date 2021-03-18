#ifndef LOCAL_DESCRIPTOR_EXTRACTOR_H
#define LOCAL_DESCRIPTOR_EXTRACTOR_H

#include "beginner_tutorials/common_include.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>

namespace enc {

class LocalDescriptorExtractor
{
private:

    static LocalDescriptorExtractor* lde; 
    LocalDescriptorExtractor() {} // private constructor makes a singleton

    static std::vector<std::vector<double> > NARFextract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

public:
    ~LocalDescriptorExtractor();
    // initialize the extractor
    static void initialize(); 
    static std::vector<std::vector<double> > extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);



    // temp function members for testing

    static void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, float voxel_x, float voxel_y, float voxel_z);

    static void normalEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int K);

    static std::vector<std::vector<double> > FPFHextract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


    
};

}

#endif