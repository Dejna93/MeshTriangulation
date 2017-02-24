#include <pcl/filters/statistical_outlier_removal.h>
#include "include/Meshing.h"


Meshing::Meshing()
{
}


Meshing::Meshing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) : input_cloud(cloud) {

}


Meshing::Meshing(Dao dao) {
    this->dao = dao;

    std::cout << "Meshing" << dao.getIntAttribute("visualisation");
}


Meshing::~Meshing()
{
}

void Meshing::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->input_cloud = cloud;
}

void Meshing::setDao(Dao dao) {
    this->dao = dao;
}

void Meshing::setIo(Io *io) {
    this->io = io;
}


void Meshing::run_calculation() {
    std::cout << "Preperation to triangluation\n";
    //dao.print();
    Cluster clustering = Cluster(dao);


    switch (dao.getIntAttribute("cluster_type")) {
        case 0 :
            clustered_cloud = clustering.clusteringEuclides(input_cloud);
            break;
        case 1:
            clustered_cloud = clustering.clusteringRegion(input_cloud);
            break;
        case 2:
            std::cout << "Plane clustering \n";
            clustered_cloud = clustering.clusteringPlane(input_cloud);
            break;
        case 3:
            clustered_cloud = clustering.clustering(input_cloud);
            break;
        default:
            std::cout << "Error wrong [cluster_type], converting whole point cloud\n";
            break;
    }

    if (!isClusteredCloud()) {
        std::cout << "Cant divide cloud to smaller subclouds, choice other [clustered_cloud]" <<
                  "\n Add whole cloud to compute\n";
        this->clustered_cloud.push_back(this->input_cloud);
    }
    Visualization vis;
    if (dao.getBoolAttribute("show_clustered")) {
        std::cout << "Trying to visualization this cloud clusters\n";
        vis.view(this->clustered_cloud);
    }

    Surface surface = Surface(dao);

    if (!this->clustered_cloud.empty()) {
        std::cout << "Start making STL from cluster.\n";
        Upsampling upsampling = Upsampling(dao);
        std::vector<pcl::PolygonMesh> meshes;
        for (size_t i = 0; i < this->clustered_cloud.size(); ++i) {
            std::cout << "\nCloud nr " << i << " computing\n";
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = noiseRemove(clustered_cloud[i]);
            if (dao.getBoolAttribute("savepcd")) {
                io->savePCD(cloud, i);
            }
            switch (dao.getIntAttribute("type_triangulation")) {
                case 0: {
                    {
                        std::cout << "\nLaplacian VTK triangulation\n";
                        // std::string lap ="/home/dejna/abaqus_plugin/CloudMesh/workspace/project/stl/" + std::to_string(i) +"lap.stl";
                        //pcl::io::savePolygonFileSTL(lap, surface.laplacianSurface(cloud));
                        pcl::PolygonMesh mesh = surface.laplacianSurface(cloud);
                        meshes.push_back(mesh);

                        io->saveSTL(i, mesh);
                        break;
                    }
                }

                case 1: {
                    //  std::string poisson = "/home/dejna/abaqus_plugin/CloudMesh/workspace/project/stl/" + std::to_string(i) +"poisson.stl";
                    //pcl::io::savePolygonFileSTL(poisson, surface.poissonSurface(cloud));
                    pcl::PolygonMesh mesh = surface.poissonSurface(cloud);
                    meshes.push_back(mesh);
                    io->saveSTL(i, mesh);
                    break;
                }

                case 2: {
                    // std::string gredy ="/home/dejna/abaqus_plugin/CloudMesh/workspace/project/stl/"+std::to_string(i)+"greedy.stl";
                    //  pcl::io::savePolygonFileSTL(gredy,surface.greedySurface(cloud));
                    pcl::PolygonMesh mesh = surface.greedySurface(cloud);
                    meshes.push_back(mesh);
                    io->saveSTL(i, mesh);
                    break;
                }
                default:
                    std::cerr << "Error set param [type_triangulation]\n";
                    return;
            }
        }

        if (dao.getIntAttribute("show_mesh")) {
            Visualization vis = Visualization();
            vis.view_mesh(meshes);
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Meshing::getCloud() {
    return input_cloud;
}

bool Meshing::isClusteredCloud() {
    return clustered_cloud.size() != 0;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
Meshing::noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int nr_k = dao.getIntAttribute("nr_k");
    double stddev = dao.getDoubleAttribute("stddev_ml");

    return noiseRemove(cloud, nr_k, stddev);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Meshing::noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int nr_k, double stddev_mlt) {
    std::cout << "Noise remove from point cloud";
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    //Set the number of nearest neighbors to use for mean distance estimation.
    sor.setMeanK(nr_k);
    //Set the standard deviation multiplier for the distance threshold calculation.
    //The distance threshold will be equal to: mean + stddev_mult * stddev.
    // Points will be classified as inlier or outlier
    //if their average neighbor distance is below or above this threshold respectively.
    sor.setStddevMulThresh(stddev_mlt);
    sor.filter(*cloud);
    return cloud;
}