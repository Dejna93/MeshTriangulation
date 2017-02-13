//
// Created by dejna on 11.02.17.
//

#ifndef MESH_TRIANGULATION_GREEDYTRIANGULATION_H
#define MESH_TRIANGULATION_GREEDYTRIANGULATION_H

#include "include/triangulation/Triangulation.h"
#include "include/Dao.h"
#include "include/triangulation/LaplacianTriangulation.h"
#include <src/main/include/Io.h>

class GreedyTriangulation : public Triangulation {

public:
    GreedyTriangulation();

    GreedyTriangulation(Dao dao);

    void process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Io &io);

private:
    void saveProccesed(Io &io);

    pcl::PolygonMesh::Ptr triangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


};


#endif //MESH_TRIANGULATION_GREEDYTRIANGULATION_H
