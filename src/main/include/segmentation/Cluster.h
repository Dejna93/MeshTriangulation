//
// Created by Damian on 2017-02-13.
//

#ifndef MESH_TRIANGULATION_EUCLIDEANCLUSTER_H
#define MESH_TRIANGULATION_EUCLIDEANCLUSTER_H
//http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_(basic)
//http://www.scielo.br/scielo.php?script=sci_arttext&pid=S0104-65002004000100005&lng=en&nrm=iso&tlng=en
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include "../Dao.h"

class Cluster {

public:
    Cluster();

    Cluster(Dao dao);

    Cluster(Dao dao, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusteringEuclides();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusteringEuclides(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusteringRegion();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusteringRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusteringPlane();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusteringPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusteringVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
private:
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    convertCluster(std::vector<pcl::PointIndices> indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<pcl::PointIndices> euclideanExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    Dao dao;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;


};

#endif //MESH_TRIANGULATION_EUCLIDEANCLUSTER_H