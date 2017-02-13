//
// Created by Damian on 2017-02-13.
//

#include "../include/segmentation/Cluster.h"


Cluster::Cluster() {
}
Cluster::Cluster(Dao dao) {
    this->dao = dao;
    this->input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
}

Cluster::Cluster(Dao dao, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->dao = dao;
    this->input_cloud = cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> EuclideanCluster::clusteringEuclides()
{
    return clustering(this->input_cloud);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
Cluster::clusteringEuclides( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);

    // Euclidean clustering object.
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    // Set cluster tolerance to
    // in several clusters, whereas big values may join objects in a same cluster).
    clustering.setClusterTolerance(dao.getDoubleAttribute("euc_cluster_tolerance"));
    // Set the minimum and maximum number of points that a cluster can have.
    clustering.setMinClusterSize(dao.getIntAttribute("euc_min_cluster_size"));
    clustering.setMaxClusterSize(dao.getIntAttribute("euc_max_cluster_size"));
    clustering.setSearchMethod(kdtree);
    clustering.setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    clustering.extract(clusters);

    return  convertCluster(clusters);
}

std::vector<pc::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringRegion()
{
    return clusteringRegion(this->input_cloud);
}

std::vector<pc::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);

    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setRadiusSearch(dao.getDoubleAttribute("reg_radius"));
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // Region growing clustering object.
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> clustering;
    clustering.setMinClusterSize(dao.getIntAttribute("reg_min_cluster"));
    clustering.setMaxClusterSize(dao.getIntAttribute("reg_max_cluster"));
    clustering.setSearchMethod(kdtree);
    clustering.setNumberOfNeighbours(dao.getIntAttribute("reg_num_neighbours"));
    clustering.setInputCloud(cloud);
    clustering.setInputNormals(normals);
    // Set the angle in radians that will be the smoothness threshold
    // (the maximum allowable deviation of the normals).
    clustering.setSmoothnessThreshold(7.0 / 180.0 * M_PI); // 7 degrees.
    // Set the curvature threshold. The disparity between curvatures will be
    // tested after the normal deviation check has passed.
    clustering.setCurvatureThreshold(dao.getDoubleAttribute("reg_curvature"));

    std::vector <pcl::PointIndices> clusters;
    clustering.extract(clusters);

    return convertCluster(clusters);
}

std::vector<pc::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringPlane()
{
    return clusteringPlane(this->input_cloud);
}

std::vector<pc::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
// Object for storing the plane model coefficients.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // Create the segmentation object.
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(cloud);
    // Configure the object to look for a plane.
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    // Use RANSAC method.
    segmentation.setMethodType(pcl::SAC_RANSAC);
    // Set the maximum allowed distance to the model.
    segmentation.setDistanceThreshold(dao.getDoubleAttribute("pl_distance"));
    // Enable model coefficient refinement (optional).
    segmentation.setOptimizeCoefficients(true);

    pcl::PointIndices inlierIndices;
    segmentation.segment(inlierIndices, *coefficients);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::convertCluster(
        std::vector <pcl::PointIndices> indices)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cluster);
    }
    return  clusters;
}