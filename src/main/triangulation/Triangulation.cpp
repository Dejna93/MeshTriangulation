
#include <pcl/features/normal_3d.h>
#include "include/triangulation/Triangulation.h"

Triangulation::Triangulation() {}

Triangulation::~Triangulation() {}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Triangulation::noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int nr_k = dao.getIntAttribute("nr_k");
    double stddev = dao.getDoubleAttribute("stddev_ml");

    return noiseRemove(cloud, nr_k, stddev);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Triangulation::noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int nr_k, double stddev_mlt) {
    std::cout << "Noise remove from point cloud";
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    //Set the number of nearest neighbors to use for mean distance estimation.
    sor.setMeanK(nr_k);
    //Set the standard deviation multiplier for the distance threshold calculation.
    //The distance threshold will be equal to: mean + stddev_mult * stddev.
    // Points will be classified as inlier or outlier
    //if their average neighbor distance is below or above this threshold respectively.
    sor.setStddevMulThresh(stddev_mlt);
    sor.filter(*cloud);
    return cloud;
}

Triangulation::Triangulation(Dao dao) {
    this->dao = dao;

}

std::vector<pcl::PointIndices> Triangulation::euclideanExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(dao.getIntAttribute("cluster_tolerance")); // 2cm
    ec.setMinClusterSize(dao.getIntAttribute("min_cluster_size"));
    ec.setMaxClusterSize(dao.getIntAttribute("max_cluster_size"));
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);


    return cluster_indices;
}

void
Triangulation::saveClusterFromSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                           std::vector<pcl::PointIndices> cluster_indices) {
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //dodanie clustra do vectora
        this->cloud_cluster.push_back(cloud_cluster);


        // Visualization visualization = Visualization();
        //visualization.view_visaulization(visualization.simpleVis(cloud_cluster));

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
                  << std::endl;

    }
}

void Triangulation::calculateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius) {

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(radius);
    ne.compute(*cloud_normals);

}


pcl::PointCloud<pcl::Normal>::Ptr
Triangulation::calculateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(radius);
    n.compute(*normals);

    return normals;
}

void Triangulation::division_to_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr copied_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setMaxIterations(100);
    seg.setMaxIterations(dao.getIntAttribute("iterations"));
    seg.setDistanceThreshold(dao.getIntAttribute("distance_threshold"));

    int i = 0, nr_points = (int) cloud->points.size();
    while (cloud->points.size() > dao.getDoubleAttribute("cloud_multipler") * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        extract.filter(*cloud_plane);

        extract.filter(*copied_cloud);
        *cloud = *copied_cloud;
    }
    // std::vector<pcl::PointIndices> cluster_indices = euclideanExtraction(cloud);
    saveClusterFromSegmentation(cloud, euclideanExtraction(cloud));

}


/*
Triangulation::Triangulation(){

}

Triangulation::~Triangulation(){}

void
Triangulation::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  this->cloud = cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Triangulation::getCloud()
{
  return this->cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Triangulation::noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int nr_k, double stddev_mult)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(basic_cloud_ptr);
  //Set the number of nearest neighbors to use for mean distance estimation.
  sor.setMeanK(nr_k);
  //Set the standard deviation multiplier for the distance threshold calculation.
  //The distance threshold will be equal to: mean + stddev_mult * stddev.
  // Points will be classified as inlier or outlier
  //if their average neighbor distance is below or above this threshold respectively.
  sor.setStddevMulThresh(stddev_mult);
  sor.filter(*cloud);
  return cloud;
}
*/