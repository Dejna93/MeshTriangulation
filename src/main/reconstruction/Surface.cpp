//
// Created by dejna on 13.02.17.
//

#include <pcl/features/normal_3d_omp.h>
#include <src/main/include/Visualization.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/concave_hull.h>
#include "include/reconstruction/Surface.h"


Surface::Surface(Dao dao) {
    this->dao = dao;
}

pcl::PolygonMesh Surface::poissonSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::Poisson<pcl::PointNormal> poisson;
    pcl::PolygonMesh triangles;


    poisson.setInputCloud(estimatedNormals(cloud));

    poisson.setDepth(dao.getIntAttribute("poisson_depth"));
    poisson.setSolverDivide(dao.getIntAttribute("poisson_solver_divide"));
    poisson.setIsoDivide(dao.getIntAttribute("poisson_iso_divide"));
    poisson.setSamplesPerNode(dao.getDoubleAttribute("poisson_samples_per_node"));
    poisson.setConfidence(dao.getBoolAttribute("poisson_confidence"));

    poisson.reconstruct(triangles);
    return triangles;
}

pcl::PolygonMesh Surface::greedySurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    //types fix
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(estimatedNormals(cloud));

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(dao.getDoubleAttribute("greedy_radius"));
    // Set typical values for the parameters
    gp3.setMu(dao.getDoubleAttribute("greedy_mu"));
    gp3.setMaximumNearestNeighbors(dao.getIntAttribute("greedy_maximum_neighbors"));
    gp3.setMaximumSurfaceAngle(dao.getDoubleAttribute("greedy_surface_angle")); // 45 degrees
    gp3.setMinimumAngle(dao.getDoubleAttribute("greedy_min_angle")); // 10 degrees
    gp3.setMaximumAngle(dao.getDoubleAttribute("greedy_max_angle")); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(estimatedNormals(cloud));
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    return triangles;
}

pcl::PolygonMesh Surface::rbfSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::MarchingCubesRBF<pcl::PointNormal> mc;
    pcl::PolygonMesh triangles;
    Visualization vis;
    mc.setIsoLevel(0);
    //mc.setGridResolution();
    mc.setPercentageExtendGrid(50);
    pcl::PointCloud<pcl::PointNormal>::Ptr test = estimatedNormals(cloud);
    //vis.view(test);
    mc.setInputCloud(estimatedNormals(cloud));
    mc.reconstruct(triangles);
    std::cout << triangles.polygons.size() << "\n";
    return triangles;
}

pcl::PointCloud<pcl::PointNormal>::Ptr Surface::estimatedNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads((unsigned int) dao.getIntAttribute("thread_num"));

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(dao.getDoubleAttribute("normal_radius"));
    ne.setKSearch(dao.getIntAttribute("normal_k"));

    if (dao.getBoolAttribute("normal_centroid") == 1) {
        Eigen::Vector4f centriod;
        compute3DCentroid(*cloud, centriod);
        ne.setViewPoint(centriod[0], centriod[1], centriod[2]);
    }

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);

    if (dao.getBoolAttribute("normal_minus") == 1) {
        for (size_t i = 0; i < cloud_normals->size(); ++i) {
            cloud_normals->points[i].normal_x *= -1;
            cloud_normals->points[i].normal_y *= -1;
            cloud_normals->points[i].normal_z *= -1;
        }
    }


    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
    concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);
    return cloud_smoothed_normals;
}

pcl::PolygonMesh Surface::laplacianSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    std::cout << "Begin Concave Hull Reconstruction...";
    boost::shared_ptr<pcl::PolygonMesh> meshIn = computeConcaveHull(cloud, 1.0);
    std::cout << "Done." << std::endl;

    std::cout << "Begin Laplacian VTKSmoothing...";
    pcl::PolygonMesh output;
    pcl::MeshSmoothingLaplacianVTK vtk;
    vtk.setInputMesh(meshIn);
    vtk.setNumIter(dao.getIntAttribute("lap_max_iter"));
    vtk.setConvergence(dao.getDoubleAttribute("lap_convergence"));
    vtk.setRelaxationFactor(dao.getDoubleAttribute("lap_relaxation_factor"));
    vtk.setFeatureEdgeSmoothing(dao.getBoolAttribute("lap_edge_smoothing"));
    vtk.setFeatureAngle(M_PI / 4);
    vtk.setBoundarySmoothing(dao.getBoolAttribute("lap_boundary_smoothing"));
    vtk.process(output);

    return output;
}

boost::shared_ptr<pcl::PolygonMesh>
Surface::computeConcaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float alpha) {

    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud(input);
    concave_hull.setAlpha(alpha);
    boost::shared_ptr<pcl::PolygonMesh> output(new pcl::PolygonMesh);
    concave_hull.reconstruct(*output);
    return (output);
}
