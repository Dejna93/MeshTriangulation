
#include "include/PoissonTriangulation.h"

PoissonTriangulation::PoissonTriangulation(){}

PoissonTriangulation::~PoissonTriangulation() {}
/*
pcl::PolygonMesh
PoissonTriangulation::calculateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius, int thread_num,
                                    int depth           , int degree       , float	samples_per_node , float scale ,
                                    int iso_divide      , bool confidence  , bool	manifold           ,
                                    bool output_polygon , int solver_divide	)
{
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads(thread_num);
  ne.setInputCloud(cloud);
  ne.setRadiusSearch(radius);

  Eigen::Vector4f centriod;
  compute3DCentroid(*cloud , centriod);
  ne.setViewPoint(centriod[0] , centriod[1] , centriod[2]);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);

  for(size_t i = 0 ; i < cloud_normals->size() ; ++i)
  {
    cloud_normals->points[i].normal_x *= -1;
    cloud_normals->points[i].normal_y *= -1;
    cloud_normals->points[i].normal_z *= -1;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);

  std::cout << "Begin poisson reconstruction \n";
  pcl::Poisson<pcl::PointNormal>poisson;
  //Set the maximum depth of the tree that will be used for surface reconstruction
  poisson.setDepth(depth);
  //Set the degree parameter.
  poisson.setDegree(degree);
  //Set the minimum number of sample points that should fall within an octree node as the octree construction is adapted to sampling density.
  //For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.
  poisson.setSamplesPerNode(samples_per_node);
  //Set the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube.
  poisson.setScale(scale);
  //Set the depth at which a block iso-surface extractor should be used to extract the iso-surface.
  poisson.setIsoDivide(iso_divide);
  //Enabling this flag tells the reconstructor to use the size of the normals as confidence information. When the flag is not enabled, all normals are normalized to have unit-length prior to reconstruction.
  poisson.setConfidence(confidence);
  //Enabling this flag tells the reconstructor to add the polygon barycenter when triangulating polygons with more than three vertices.
  //Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)
  //poisson.setMinifold(manifold);
  //Enabling this flag tells the reconstructor to output a polygon mesh (rather than triangulating the results of Marching Cubes).
  poisson.setOutputPolygons(output_polygon);
  //Set the the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation.
  //For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.
  poisson.setSolverDivide(solver_divide);

  poisson.setInputCloud(cloud_smoothed_normals);

  pcl::PolygonMesh mesh;
  poisson.reconstruct(mesh);

  return mesh;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
PoissonTriangulation::splitCloudToClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr copied_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

  seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(1);

  int i = 0, nr_points = (int)cloud->points.size();
  while (cloud->points.size() > 1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
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

    extract.filter(*cloud_plane);

    extract.filter(*copied_cloud);
		*cloud = *copied_cloud;
  }
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]); //
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      //dodanie clustra do vectora
      cloud_clusters.push_back(cloud_cluster);

      //calculateMesh
      //writeSTL

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    //writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //
  }
  return cloud_clusters;
}

void PoissonTriangulation::makeSTLfromClusters(  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = splitCloudToClusters(cloud);

   if(clusters->size() > 0 )
   {
     for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
     {
       pcl::PolygonMesh mesh = calculateMesh(cluster , paramsTODO);
       //ioManager.saveSTL();

     }
   }
}
*/
