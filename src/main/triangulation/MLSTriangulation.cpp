//
// Created by dejna on 11.02.17.
//

#include "include/triangulation/MLSTriangulation.h"

MLSTriangulation::MLSTriangulation() {

}

MLSTriangulation::MLSTriangulation(Dao dao) : Triangulation(dao) {
    this->dao = dao;
}

void MLSTriangulation::process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Io &io) {
    std::cout << "Moving Least Squeares \n";
    calculateNormals(cloud, dao.getDoubleAttribute("radius_norm_small"));

    std::cout << "Noise remove from cloud \n";
    noiseRemove(cloud);

    std::cout << "Divide subclouds to clusters \n";
    division_to_clusters(cloud);

    std::cout << "Smoothing and save to STL\n";

    saveProccesedCloud(io);
}

pcl::PolygonMesh MLSTriangulation::smoothingMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    /*
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal> mls_points;

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.1);
    mls.process(mls_points);
*/
}

void MLSTriangulation::saveProccesedCloud(Io &io) {

}

