//
// Created by dejna on 11.02.17.
//

#include <pcl/surface/gp3.h>
#include "include/triangulation/GreedyTriangulation.h"


GreedyTriangulation::GreedyTriangulation() {}

GreedyTriangulation::GreedyTriangulation(Dao dao) : Triangulation(dao) {
    this->dao = dao;
}

void GreedyTriangulation::process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Io &io) {
    // std::cout << "Greedy triangulation  \n";
    calculateNormal(cloud, 20);
    //calculateNormals(cloud, dao.getDoubleAttribute("radius_norm_norm"));

    std::cout << "Noise remove from cloud \n";
    noiseRemove(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_save = cloud;
    Visualization visualization = Visualization();
    visualization.view_visaulization(visualization.simpleVis(cloud));

    std::cout << "Divide subclouds to clusters \n";
    division_to_clusters(cloud);

    //   if (this->cloud_cluster.size() == 0){
    //    std::cout << "Adding to cluster\n";
    //   this->cloud_cluster.push_back(cloud_save);
    // }
    std::cout << "Smoothing and save to STL\n";

    saveProccesed(io);
    std::cout << "Saved\n";
}

pcl::PolygonMesh::Ptr GreedyTriangulation::triangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::cout << "Smoothing greedy \n";
    pcl::PointCloud<pcl::Normal>::Ptr normals = calculateNormal(cloud, 2);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh::Ptr triangles;

    gp3.setSearchRadius(1.5);

    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(700);
    gp3.setMaximumSurfaceAngle(M_PI * 2); // 45 degrees
    gp3.setMinimumAngle(0); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI); // 120 degrees
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);

    return triangles;
}

void GreedyTriangulation::saveProccesed(Io &io) {
    for (auto it = this->cloud_cluster.begin();
         it != cloud_cluster.end(); ++it)//(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : this->cloud_cluster)
    {
        auto index = std::distance(cloud_cluster.begin(), it);
        std::cout << "Cloud number " << index << std::endl;
        if (io.getDao().getIntAttribute("savepcd") == 1) {
            //SavePCD
            io.savePCD(*it, index);

        }

        std::cout << "Triangulation greedy \n";
        //triangulation
        pcl::PolygonMesh::Ptr output_mesh = triangulation(*it);
        LaplacianTriangulation laplacianSmoothing = LaplacianTriangulation();
        //smoothing
        output_mesh = laplacianSmoothing.smoothingMesh(output_mesh);
        //TODO IO HANDLE SAVE STL
        io.saveSTL(index, *output_mesh);
        //pcl::io::savePolygonFileSTL("meshsmoothing.stl", output_mesh);

    }
}