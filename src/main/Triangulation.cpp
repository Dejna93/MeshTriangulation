

/*
Triangulation::Triangulation(){

}

Triangulation::~Triangulation(){}

void
Triangulation::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  this->cloud = cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Triangulation::getCloud()
{
  return this->cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Triangulation::noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int nr_k, double stddev_mult)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(basic_cloud_ptr);
  //Set the number of nearest neighbors to use for mean distance estimation.
  sor.setMeanK(nr_k);
  //Set the standard deviation multiplier for the distance threshold calculation.
  //The distance threshold will be equal to: mean + stddev_mult * stddev.
  // Points will be classified as inlier or outlier
  //if their average neighbor distance is below or above this threshold respectively.
  sor.setStddevMulThresh(stddev_mult);
  sor.filter(*cloud);
  return cloud;
}
*/