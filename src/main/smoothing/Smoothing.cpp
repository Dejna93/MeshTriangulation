//
// Created by dejna on 12.02.17.
//

#include "include/smoothing/Smoothing.h"

Smoothing::Smoothing(Dao dao) {
    this->dao = dao;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Smoothing::smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::MovingLeastSquares <pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(dao.getDoubleAttribute("mls_radius"));
    //mls.setSqrGaussParam(dao.getDoubleAttribute("mls_sqrt_gauss"));
    mls.setPolynomialFit(dao.getBoolAttribute("mls_polygon_fit"));
    // mls.setPolynomialOrder(dao.getIntAttribute("mls_polygon_order"));
    // mls.setUpsamplingMethod();
    // mls.setPointDensity();
    //  mls.setUpsamplingRadius();
    // mls.setUpsamplingStepSize();
    // mls.setDilationIterations();
    //  mls.setDilationVoxelSize();
    mls.setComputeNormals(dao.getBoolAttribute("mls_compute_normals"));

    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    mls.setSearchMethod(tree);

    mls.process(*cloud_smoothed);

    return cloud_smoothed;
}