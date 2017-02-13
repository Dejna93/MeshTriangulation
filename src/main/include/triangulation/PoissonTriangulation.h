#ifndef POISSONTRIANGULATION_H
#define POISSONTRIANGULATION_H

#include <iostream>

#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_lib_io.h>

#include <boost/thread.hpp>


#include "include/triangulation/Triangulation.h"
#include "include/Dao.h"
#include <src/main/include/Io.h>

class PoissonTriangulation : public Triangulation
{
public:
    PoissonTriangulation();

    PoissonTriangulation(Dao dao);

    ~PoissonTriangulation();

    void proccess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Io &io);


private:
    Dao dao;

    void saveProcesedCloud(Io &io);

    pcl::PolygonMesh smoothingMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);



/*
    pcl::PolygonMesh PoissonTriangulation::calculateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius ,
                                                         int thread_num , int depth , int degree,
                                                         float samples_per_node, float scale, int iso_divide,
                                                         bool confidence, bool manifold, bool output_polygon,
                                                         int solver_divide);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    splitCloudToClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void makeSTLfromClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    */
};
#endif //M