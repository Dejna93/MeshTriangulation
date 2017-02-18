#include "include/Visualization.h"



Visualization::Visualization() : text_id(0)
{
}


Visualization::~Visualization()
{
}


void
Visualization::view_visaulization(const  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}




boost::shared_ptr<pcl::visualization::PCLVisualizer>
Visualization::simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addText("CLOUD TEST", 10, 10);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
Visualization::simpleVis(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointNormal>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addText("CLOUD TEST", 10, 10);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return viewer;
}



boost::shared_ptr<pcl::visualization::PCLVisualizer>
Visualization::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
Visualization::customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
Visualization::normalsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}




boost::shared_ptr<pcl::visualization::PCLVisualizer>
Visualization::viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);

	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}

void
Visualization::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		this->text_id = 0;
	}
}

void
Visualization::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

		char str[512];
		sprintf(str, "text#%03d", this->text_id++);
		viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}

void Visualization::view_mesh(const pcl::PolygonMesh &mesh) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(mesh, "meshes", 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}

void Visualization::view_mesh(const pcl::PolygonMesh::Ptr &mesh) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(*mesh, "meshes", 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}

void Visualization::view(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalVis(clouds);
    //viewer->setSize(640, 480);
    viewer->setSize(800, 600);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(150, 100, 100, -0.6, 0.8, 0.1, 0.9, -0.3, -0.3);

    while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}

void Visualization::view(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud);
    viewer->setBackgroundColor(0, 0, 0);


    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }

}



boost::shared_ptr<pcl::visualization::PCLVisualizer>
Visualization::normalVis(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0, "axis", 0);


    int index = 1;
    int color = 50;
    int ypoz = 590;
    int xpoz = 10;
	for (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud : clouds) {

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, color + 10, color += 20,
                                                                                     color += 5);
        viewer->addPointCloud(cloud, single_color, std::to_string(index));
		std::string text = "Cloud nr=" + std::to_string(index) + ",size=" + std::to_string(cloud->size());
        viewer->addText(text, xpoz, ypoz -= 10);
        if (index == 50) {
            xpoz = 670;
            ypoz = 590;
        }
		index++;
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

    }
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

	return viewer;
}

void Visualization::view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	view(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> {cloud});
}

