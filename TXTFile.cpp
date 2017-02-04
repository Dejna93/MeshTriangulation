#include "TXTFile.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
pcl::TXTFile::TXTFile() {

}

pcl::TXTFile::~TXTFile() {

}

std::vector<PointXYZDEBUG> pcl::TXTFile::loadTxtFile(std::string &file_name) {
	std::ifstream file;
	std::cout << "Loading from txt file \n";
	pcl::TXTFile::loadFile(file_name, file);
	std::cout << "\n";
	pcl::TXTFile::loadCloud();
	return points;
	
}

int pcl::TXTFile::loadFile(std::string &file_name, std::ifstream & file) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//FOR DEBUG DELETE IN RELEASE VERSION
	file_name = "dane.txt";
	std::cout << "File to open :" << file_name <<"\n";
	file.open(file_name, std::ios::in);

	if (file.is_open()) {
			while(!file.eof()) {
				PointXYZDEBUG point;

				file >> point;
				points.push_back(point);
		}
	}
	return 1;

}

std::istream & pcl::operator >> (std::istream & input, PointXYZDEBUG & point)
{
	// Zczytywanie lini z pliku i prasowanie jej do structury PointXYZ
	PointXYZS points;

	std::string line;  std::getline(input, line);
	std::stringstream stream(line);

	stream >> points.x >> points.y >> points.z;

	point.x = stof(points.x);
	point.y = stof(points.y);
	point.z = stof(points.z);

	std::cout << point << "\n";
	return input;
}


std::ostream& pcl::operator<<(std::ostream & out, const PointXYZDEBUG &p) {
	using namespace std;
	out << "point	"
		<< setw(4) << left << p.x << " "
		<< setw(4) << left << p.y << " "
		<< setw(4) << left << p.z << " ";
	return out;
}

//--------------------------------

int pcl::TXTFile::loadCloud() {

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>cloud;
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("bun0.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* the data should be available in cloud

	/*cloud.width = 5;
	cloud.height = 5;
	cloud.is_dense = false;
	cloud.points.resize(points.size());

	for (size_t i = 0; i < cloud.points.size(); i++) {
		cloud.points[i].x = points.at(i).x;
		cloud.points[i].y = points.at(i).y;
		cloud.points[i].z = points.at(i).z;
	}
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouder(new pcl::PointCloud<pcl::PointXYZ>);
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);



	return 1;
}