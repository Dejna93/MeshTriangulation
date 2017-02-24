//
// Created by Damian on 2017-02-13.
//

#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <src/main/include/Visualization.h>
#include "../include/segmentation/Cluster.h"


#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/normal_3d_omp.h>


Cluster::Cluster() {
}

Cluster::Cluster(Dao dao) {
    this->dao = dao;
}

Cluster::Cluster(Dao dao, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->dao = dao;
    this->input_cloud = cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringEuclides() {
    return clusteringEuclides(this->input_cloud);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
Cluster::clusteringEuclides(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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

    return convertCluster(clusters, cloud);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringRegion() {
    return clusteringRegion(this->input_cloud);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

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

    std::vector<pcl::PointIndices> clusters;
    clustering.extract(clusters);

    return convertCluster(clusters, cloud);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringPlane() {
    return clusteringPlane(this->input_cloud);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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

        //xtract.filter(*copied_cloud);
        *cloud = *cloud_plane;
    }

    std::cout << cloud->points.size() << "\n";
    return convertCluster(euclideanExtraction(cloud), cloud);
}


/*
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

// Object for storing the plane model coefficients.
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
// Create the segmentation object.
std::cout << cloud->points.size();
pcl::SACSegmentation<pcl::PointXYZ> segmentation;
//segmentation.setInputCloud(cloud);
// Configure the object to look for a plane.
segmentation.setModelType(pcl::SACMODEL_PLANE);
// Use RANSAC method.
segmentation.setMethodType(pcl::SAC_RANSAC);
// Set the maximum allowed distance to the model.
segmentation.setDistanceThreshold(dao.getDoubleAttribute("pl_distance"));
// Enable model coefficient refinement (optional).
segmentation.setOptimizeCoefficients(true);
// Max interations
segmentation.setMaxIterations(dao.getIntAttribute("pl_max_iter"));

pcl::PointIndices::Ptr inlierIndices;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

int i = 0, nr_points = (int) cloud->points.size();
while (cloud->points.size() > dao.getDoubleAttribute("cloud_multipler") * nr_points) {
    // Segment the largest planar component from the remaining cloud
    segmentation.setInputCloud(cloud);
    segmentation.segment(*inlierIndices, *coefficients);
    if (inlierIndices->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
    }
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inlierIndices);
    extract.setNegative(false);

    extract.filter(*cloud_plane);
}

std::cout <<"Ending\n";
return convertCluster(euclideanExtraction(cloud_plane),this->input_cloud);
}
*/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::convertCluster(
        std::vector<pcl::PointIndices> indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin();
         it != indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }
    return clusters;
}

std::vector<pcl::PointIndices> Cluster::euclideanExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clusteringVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(
            new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    Visualization vis = Visualization();
    // Downsample the cloud using a Voxel Grid class
    //std::cerr << "Downsampling...\n";
    //pcl::VoxelGrid<pcl::PointXYZ> vg;
    //vg.setInputCloud (cloud);
    //vg.setLeafSize (80.0, 80.0, 80.0);
    //vg.setDownsampleAllData (true);
    //vg.filter (*cloud_out);
    //std::cerr << ">> Done: " << cloud_out->points.size () << " points\n";

    // Set up a Normal Estimation class and merge data in cloud_with_normals
    std::cout << "Computing normals...\n";
    //pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setNumberOfThreads(4);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(search_tree);
    ne.setRadiusSearch(dao.getDoubleAttribute("cl_radius"));
    ne.compute(*cloud_with_normals);
    std::cout << ">> Done: " << cloud_with_normals->points.size() << " ms\n";
    vis.view(cloud_with_normals);

    // Set up a Conditional Euclidean Clustering class
    std::cerr << "Segmenting to clusters...\n";
    pcl::ConditionalEuclideanClustering<pcl::PointNormal> cec(true);
    cec.setInputCloud(cloud_with_normals);
    // cec.setConditionFunction(&customRegionGrowing);
    cec.setClusterTolerance(500.0);
    cec.setMinClusterSize(cloud_with_normals->points.size() / 1000);
    cec.setMaxClusterSize(cloud_with_normals->points.size() / 5);
    cec.segment(*clusters);
    cec.getRemovedClusters(small_clusters, large_clusters);
    std::cerr << ">> Done: " << " ms\n";

    return std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
// Normal estimation
    //std::cout << dao.getIntAttribute("thread_num") << " " << dao.getDoubleAttribute("euc_cluster_tolerance") << " " <<dao.getIntAttribute("euc_min_cluster_size") << "\n";
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(dao.getIntAttribute("thread_num"));
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree_n);
    ne.setRadiusSearch(dao.getIntAttribute("normal_radius"));
    ne.compute(*cloud_normals);
    std::cout << "Estimated the normals" << std::endl;

    // Creating the kdtree object for the search method of the extraction
    boost::shared_ptr<pcl::KdTree<pcl::PointXYZ> > tree_ec(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    tree_ec->setInputCloud(cloud);

    // Extracting Euclidean clusters using cloud and its normals
    std::vector<int> indices;
    std::vector<pcl::PointIndices> cluster_indices;
    //const float tolerance = 2.0f; // 50cm tolerance in (x, y, z) coordinate system
    const double eps_angle = 5 * (M_PI / 180.0); // 5degree tolerance in normals
    //const unsigned int min_cluster_size = 25;

    pcl::extractEuclideanClusters(*cloud, *cloud_normals, dao.getDoubleAttribute("euc_cluster_tolerance"), tree_ec,
                                  cluster_indices, eps_angle, dao.getIntAttribute("euc_min_cluster_size"));

    std::cout << "No of clusters formed are " << cluster_indices.size() << std::endl;

    return convertCluster(cluster_indices, cloud);
}

