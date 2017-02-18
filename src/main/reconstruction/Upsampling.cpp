//
// Created by dejna on 13.02.17.
//

#include <src/main/include/Visualization.h>
#include "include/reconstruction/Upsampling.h"

Upsampling::Upsampling(Dao dao) {
    this->dao = dao;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Upsampling::process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::cout << "\nBefore upsampling" << cloud->points.size();
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setSearchRadius(1.1);
    // filter.setPolynomialFit(true);
    //filter.setPolynomialOrder(2);
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    filter.setUpsamplingRadius(2);
    filter.setUpsamplingStepSize(2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());
    filter.process(*cloud_smoothed);
    std::cout << "\nupsampling" << cloud_smoothed->points.size();

    return cloud_smoothed;
}