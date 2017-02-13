#ifndef VISUALIZATION_H
#define VISUALIZATION_H
#include <boost/thread/thread.hpp>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

class Visualization
{
public:
	Visualization();
	~Visualization();

	void view_visaulization(const boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

	void view_mesh(const pcl::PolygonMesh &mesh);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2);

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis();



private:

	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

	void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void);

	unsigned int text_id;

};

#endif //M