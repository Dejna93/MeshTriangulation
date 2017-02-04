#pragma once
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <boost/numeric/ublas/vector.hpp>


#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>

using namespace std;

class Meshing
{
public:
	Meshing();
	~Meshing();

	void splitCluster();

private:
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	boost::numeric::ublas::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr calculateSurfaceWithRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius = 0.05);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filteringCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::SACSegmentation<pcl::PointXYZ> * initSegmentation(bool optimaze=true, int num_iteration = 100, int distance_threshold=1 );
	pcl::PointCloud<pcl::PointXYZ>::Ptr extractIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double iteration_multipler = 1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k=50);

	pcl::PointCloud<pcl::Normal>::Ptr computeNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

	void meshPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius =2);
	void meshLaplacian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , float alpha);


};

