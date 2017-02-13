//
// Created by dejna on 12.02.17.
//

#ifndef MESH_TRIANGULATION_MLSMOOTHING_H
#define MESH_TRIANGULATION_MLSMOOTHING_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

#include "include/Dao.h"


class Smoothing {
public:

    Smoothing(Dao dao);

    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
    Dao dao;
};

#endif //MESH_TRIANGULATION_MLSMOOTHING_H


