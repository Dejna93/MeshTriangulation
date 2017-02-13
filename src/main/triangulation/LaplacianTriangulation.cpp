//
// Created by dejna on 11.02.17.
//


#include "include/triangulation/LaplacianTriangulation.h"

LaplacianTriangulation::LaplacianTriangulation() {

}

LaplacianTriangulation::LaplacianTriangulation(Dao dao) : Triangulation(
        dao) {
    this->dao = dao;

}


void LaplacianTriangulation::process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Io &io) {
    std::cout << "Calculate normals surface \n";
    calculateNormals(cloud, dao.getDoubleAttribute("radius_norm_small"));
    calculateNormals(cloud, dao.getDoubleAttribute("radius_norm_norm"));

    //with dao params
    std::cout << " Noise remove from cloud \n";
    noiseRemove(cloud);

    std::cout << "Divide subclouds to clusters \n";
    division_to_clusters(cloud);
    std::cout << "Smoothing and save to STL \n";
    saveProccesedCloud(io);

}


pcl::PolygonMesh LaplacianTriangulation::smoothingMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::cout << "Begin Concave Hull Reconstruction...";
    boost::shared_ptr<pcl::PolygonMesh> meshIn = computeConcaveHull(cloud, 1.0);
    std::cout << "Done." << std::endl;

    std::cout << "Begin Laplacian VTKSmoothing...";
    pcl::PolygonMesh output;
    pcl::MeshSmoothingLaplacianVTK vtk;
    vtk.setInputMesh(meshIn);
    vtk.setNumIter(400000);
    vtk.setConvergence(0.0001);
    vtk.setRelaxationFactor(0.0001);
    vtk.setFeatureEdgeSmoothing(true);
    vtk.setFeatureAngle(M_PI / 4);
    vtk.setBoundarySmoothing(true);
    vtk.process(output);

    return output;
}


boost::shared_ptr<pcl::PolygonMesh> LaplacianTriangulation::computeConcaveHull(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float alpha) {

    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud(input);
    concave_hull.setAlpha(alpha);
    boost::shared_ptr<pcl::PolygonMesh> output(new pcl::PolygonMesh);
    concave_hull.reconstruct(*output);
    return (output);
}

void LaplacianTriangulation::saveProccesedCloud(Io &io) {
    std::cout << "Calculate stl for clusters \n";
    for (auto it = this->cloud_cluster.begin();
         it != cloud_cluster.end(); ++it)//(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : this->cloud_cluster)
    {
        auto index = std::distance(cloud_cluster.begin(), it);
        std::cout << "Cloud number " << index << std::endl;
        if (io.getDao().getIntAttribute("savepcd") == 1) {
            //SavePCD
            io.savePCD(*it, index);

        }

        std::cout << "Smoothing \n";

        pcl::PolygonMesh output_mesh = smoothingMesh(*it);

        //TODO IO HANDLE SAVE STL
        io.saveSTL(index, output_mesh);
        //pcl::io::savePolygonFileSTL("meshsmoothing.stl", output_mesh);

    }
}

pcl::PolygonMesh::Ptr LaplacianTriangulation::smoothingMesh(pcl::PolygonMesh::Ptr mesh) {
    pcl::PolygonMesh output;
    pcl::MeshSmoothingLaplacianVTK vtk;

    vtk.setInputMesh(mesh);
    vtk.setNumIter(2000);
    vtk.setConvergence(0.0001);
    vtk.setRelaxationFactor(0.0001);
    vtk.setFeatureEdgeSmoothing(true);
    vtk.setFeatureAngle(M_PI / 5);
    vtk.setBoundarySmoothing(true);
    vtk.process(output);
}
