#pragma once
#include <vector>
class PoissonTriangulation : public Triangulation
{
public:
    PoissonTriangulation();

    ~PoissonTriangulation();

    pcl::PolygonMesh PoissonTriangulation::calculateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius = 2.0,
                                                         int thread_num = 4, int depth = 9, int degree,
                                                         float samples_per_node, float scale, int iso_divide,
                                                         bool confidence, bool manifold, bool output_polygon,
                                                         int solver_divide);

    void makeSTLfromClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};
