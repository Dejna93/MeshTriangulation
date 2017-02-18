
#ifndef MESHING_H
#define MESHING_H

#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>

#include <iostream>


#include "Dao.h"
#include "include/segmentation/Cluster.h"
#include "include/reconstruction/Upsampling.h"
#include "include/reconstruction/Surface.h"

#include "include/Io.h"
#include "Visualization.h"

using namespace std;

class Meshing
{
public:
	Meshing();

    Meshing(Dao dao);

	Meshing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	~Meshing();

	void setDao(Dao dao);

    void setIo(Io *io);

    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void run_calculation();

	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();


private:

    Dao dao;
    Io *io;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_cloud;

    bool isClusteredCloud();

    pcl::PointCloud<pcl::PointXYZ>::Ptr noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int nr_k, double stddev_mlt);


};

#endif //M