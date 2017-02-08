/* \author Geoffrey Biggs */


#include <iostream>

#include "include/Visualization.h"
#include "include/Io.h"
#include "include/Meshing.h"

#include <pcl/surface/gp3.h>


#include <pcl/surface/concave_hull.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>


unsigned int text_id = 0;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}

void showSomebean(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr copied_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cout << " Size of cloud " << cloud->points.size() << "\n";
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(1);

	int i = 0, nr_points = (int)cloud->points.size();
	while (cloud->points.size() > 1 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*copied_cloud);
		*cloud = *copied_cloud;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(2); // 2cm
	ec.setMinClusterSize(5);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int j = 0;
	cout << "halo \n";
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
		j++;
	}



}
boost::shared_ptr<pcl::PolygonMesh> computeConcaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float alpha)
{
	pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
	concave_hull.setInputCloud(input);
	concave_hull.setAlpha(alpha);
	boost::shared_ptr<pcl::PolygonMesh> output(new pcl::PolygonMesh);
	concave_hull.reconstruct(*output);
	return (output);
}

void laplacianMeshSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud , float alpha) {

	std::cout << "Begin Concave Hull Reconstruction...";
	boost::shared_ptr<pcl::PolygonMesh> meshIn = computeConcaveHull(cloud, alpha);
	std::cout << "Done." << std::endl;

	std::cout << "Begin Laplacian VTKSmoothing...";
	pcl::PolygonMesh output;
	pcl::MeshSmoothingLaplacianVTK vtk;
	vtk.setInputMesh(meshIn);
	vtk.setNumIter(40000);
	vtk.setConvergence(0.0001);
	vtk.setRelaxationFactor(0.0001);
	vtk.setFeatureEdgeSmoothing(true);
	vtk.setFeatureAngle(M_PI / 5);
	vtk.setBoundarySmoothing(true);
	vtk.process(output);
	std::cout << "Done." << std::endl;
	pcl::io::savePolygonFileSTL("meshsmoothing.stl", output);
}
// --------------
// -----Main-----
// --------------
int
main(int argc, char** argv)
{
    Io ioManager = Io();
    if (ioManager.input(argc, argv)) {
        ioManager.printStack();
        Meshing meshCreator = Meshing(ioManager.getOptionDao(), ioManager.getFilterDao(), ioManager.getPoissonDao());
        meshCreator.setInputCloud(ioManager.loadPCD());
        meshCreator.run_calculation();
    }
    return 0;

}
