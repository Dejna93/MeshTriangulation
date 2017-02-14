
#ifndef MESHING_H
#define MESHING_H

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
#include <iostream>

#include "Dao.h"

#include "triangulation/PoissonTriangulation.h"
#include "triangulation/LaplacianTriangulation.h"
#include "triangulation/GreedyTriangulation.h"
#include "Visualization.h"

using namespace std;

class Meshing
{
public:
	Meshing();

    Meshing(Dao dao);

	Meshing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    Meshing(Io &io);
	~Meshing();

	void setDao(Dao dao);

    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void run_calculation();

	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();


private:

    Dao dao;
    Io io;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;


};

#endif //M