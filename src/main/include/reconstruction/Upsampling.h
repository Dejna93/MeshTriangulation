//
// Created by dejna on 13.02.17.
//

#ifndef MESH_TRIANGULATION_UPSAMPLING_H
#define MESH_TRIANGULATION_UPSAMPLING_H

#include <pcl/surface/mls.h>

#include "../Dao.h"

class Upsampling {

public:
    Upsampling(Dao dao);

    pcl::PointCloud<pcl::PointXYZ>::Ptr process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


private:
    Dao dao;

};

#endif //MESH_TRIANGULATION_UPSAMPLING_H
