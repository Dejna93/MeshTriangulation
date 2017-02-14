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


Meshing::Meshing(Io &io) {
    this->dao = io.getDao();
    this->io = io;

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



void Meshing::run_calculation() {
    std::cout << "running calculation \n" << this->dao.getIntAttribute("visualisation") << std::endl;
    dao.print();

    if (dao.getIntAttribute("visualisation") == 1) {
        std::cout << "Visualization \n";
        Visualization visualization = Visualization();
        visualization.view_visaulization(visualization.simpleVis(this->input_cloud));
    }

    if (this->dao.getIntAttribute("smoothing") == 0) {
        //POISSON
        std::cout << " POISSON MESH \n";
        PoissonTriangulation poissonTriangulation = PoissonTriangulation(dao);
        //poissonTriangulation.noiseRemove(this->input_cloud);
        // poissonTriangulation.division_to_clusters(this->input_cloud);
        poissonTriangulation.proccess(this->input_cloud, this->io);
    }
    if (this->dao.getIntAttribute("smoothing") == 1) {
        LaplacianTriangulation laplacianTriangulation = LaplacianTriangulation(dao);

        laplacianTriangulation.process(this->input_cloud, this->io);
    }
    if (this->dao.getIntAttribute("smoothing") == 2) {
        std::cout << "Greedy Triangulation \n";
        GreedyTriangulation greedyTriangulation = GreedyTriangulation();

        greedyTriangulation.process(this->input_cloud, this->io);
    }


}

pcl::PointCloud<pcl::PointXYZ>::Ptr Meshing::getCloud() {
    return input_cloud;
}



/*

pcl::PointCloud<pcl::PointXYZ>::Ptr Meshing::calculateSurfaceWithRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius)
{
	this->ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	this->ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	this->ne.setRadiusSearch(radius);
	this->ne.compute(*cloud_normals);

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Meshing::noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(k);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	return cloud;
}


pcl::SACSegmentation<pcl::PointXYZ> * Meshing::initSegmentation(bool optimaze, int num_iteration, int distance_threshold )
{
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(optimaze);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(num_iteration);
	seg.setDistanceThreshold(distance_threshold);

	return &seg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Meshing::extractIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double iteration_multipler) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr copied_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::SACSegmentation<pcl::PointXYZ> seg = initSegmentation();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

	int points_size = (int)cloud->points.size();

	while (cloud->points.size() > iteration_multipler * points_size) {
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*copied_cloud);
		*cloud = *copied_cloud;
		return cloud;
	}

}



pcl::PointCloud<pcl::PointXYZ>::Ptr Meshing::filteringCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::cout << "begin passthrough filter" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.filter(*filtered);
	std::cout << "passthrough filter complete" << std::endl;
	return cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr Meshing::computeNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	this->ne.compute(*cloud_normals);
	std::cout << "normal estimation complete" << std::endl;
	std::cout << "reverse normals' direction" << std::endl;

	for (size_t i = 0; i < cloud_normals->size(); ++i) {
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}
	return cloud_normals;
}

void Meshing::meshPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius)
{
	std::cout << "Begin meshing poisson \n";

	this->ne.setInputCloud(cloud);
	this->ne.setRadiusSearch(radius);
	
	Eigen::Vector4f centroid;
	compute3DCentroid(*cloud, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	computeNormals(cloud_normals);

	std::cout << "combine points and normals" << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
	concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);


	cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(7);
	poisson.setInputCloud(cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);
	pcl::io::savePolygonFileSTL("data.stl", mesh);
}
*/

