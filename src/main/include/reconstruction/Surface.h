//
// Created by dejna on 13.02.17.
//

#ifndef MESH_TRIANGULATION_SURFACE_H
#define MESH_TRIANGULATION_SURFACE_H

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_rbf.h>

#include "../Dao.h"

class Surface {

public:
    Surface(Dao dao);

    pcl::PolygonMesh poissonSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PolygonMesh poissonSurface(pcl::PointCloud<pcl::Normal>::Ptr cloud);

    pcl::PolygonMesh rbfSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PolygonMesh greedySurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
    Dao dao;

    pcl::PointCloud<pcl::Normal>::Ptr estimatedNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif //MESH_TRIANGULATION_SURFACE_H
