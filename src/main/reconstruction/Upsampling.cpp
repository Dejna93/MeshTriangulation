//
// Created by dejna on 13.02.17.
//

#include "include/reconstruction/Upsampling.h"

Upsampling::Upsampling(Dao dao) {
    this->dao = dao;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Upsampling::process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // Object for searching.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    filter.setSearchMethod(kdtree);
    // Use all neighbors in a radius of 3cm
    filter.setSearchRadius(dao.getDoubleAttribute("up_search_radius")); //0.03 - 3 cm
    // Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    // and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // Radius around each point, where the local plane will be sampled.
    filter.setUpsamplingRadius(dao.getDoubleAttribute("up_sampling_radius"));
    // Sampling step size. Bigger values will yield less (if any) new points.
    filter.setUpsamplingStepSize(dao.getDoubleAttribute("up_sampling_step"));

    filter.process(*cloud);

    return cloud;
}