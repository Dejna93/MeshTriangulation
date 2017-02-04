#include "Io.h"



Io::Io()
{
}


Io::~Io()
{
}


int Io::nr_from_file(std::string filepath) {
//	std::regex wzorzec("([0 - 9])\d+");
	//std::string result = "";
	//if (std::regex_search(filepath, result, wzorzec))
	//{
	//	return std::stoi(result);
	//}
	return 0;
}

int Io::writeToPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number)
{
	pcl::PCDWriter writer;
	return writer.write<pcl::PointXYZ>(filename, *cloud, false); //*
}

int writeToPCDBinary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number)
{
	pcl::PCDWriter writer;
	return writer.writeBinary<pcl::PointXYZ>(filename, *cloud); //*
}
