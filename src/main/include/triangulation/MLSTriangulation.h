//
// Created by dejna on 11.02.17.
//

#ifndef MESH_TRIANGULATION_MLSTRIANGULATION_H
#define MESH_TRIANGULATION_MLSTRIANGULATION_H

#include "include/triangulation/Triangulation.h"
#include "include/Dao.h"
#include "include/Io.h"


class MLSTriangulation : public Triangulation {

public:
    MLSTriangulation();

    MLSTriangulation(Dao dao);

    void process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Io &io);

private:
    void saveProccesedCloud(Io &io);

    pcl::PolygonMesh smoothingMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


};


#endif //MESH_TRIANGULATION_MLSTRIANGULATION_H
