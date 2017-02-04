#pragma once
#include <string>
#include <regex>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

#include <boost/filesystem.hpp>
using namespace std;


class Io
{
public:
	Io();
	~Io();

private:
	int nr_from_file(string filepath);
	int writeToPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number);
	int writeToPCDBinary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number);
};

