#pragma once

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common_headers.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
class Mesh:
{
public:
    Mesh();
    ~Mesh();

  //  virtual pcl::PolygonMesh calculateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius) = 0 ;

    virtual std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> splitCloudToClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) = 0 ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr noiseRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void setCloud(pcl::PointCloud<pcl::PointXYZ)>::Ptr cloud, int 	nr_k ,	double stddev_mult);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

}
