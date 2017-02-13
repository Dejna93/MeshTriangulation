//
// Created by dejna on 11.02.17.
//

#ifndef LAPLACIANTRIANGULATION_H
#define LAPLACIANTRIANGULATION_H

#include <pcl/surface/concave_hull.h>
#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>

#include "include/triangulation/Triangulation.h"
#include "include/Dao.h"
#include <src/main/include/Io.h>

class LaplacianTriangulation : public Triangulation {

public:

    LaplacianTriangulation();

    LaplacianTriangulation(Dao dao);

    void process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Io &io);

    pcl::PolygonMesh::Ptr smoothingMesh(pcl::PolygonMesh::Ptr mesh);

private:

    void saveProccesedCloud(Io &io);

    pcl::PolygonMesh smoothingMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    boost::shared_ptr<pcl::PolygonMesh>
    computeConcaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float alpha);

};


#endif //MESH_TRIANGULATION_LAPLACIANTRIANGULATION_H
