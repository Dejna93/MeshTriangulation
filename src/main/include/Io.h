#pragma once
#include <string>
#include <regex>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
using namespace std;


class Io
{
public:
	Io();
	~Io();

	int input(int argc, char **argv);

	// SETTERS and GETTERS
	int setFilepath(std::string filepath);

	void setShowVisualisation(int visualisation = 0);

	void setSmoothinMethod(int method = 0);

	void setSavingPcd(int saving = 0);

	void setFiltering(int filtering = 0);

	boost::filesystem::path getFilepath();

	int getSmoothingMethod();

	bool isSavePCD();

	bool isFiltering();

private:
	boost::filesystem::path filepath;
	bool show_visualisation;
	int smoothing_method; // 0 - poisson 1 - vtk smoothing 2 - other to impl
	bool save_pcd;
	bool filtering;


	void printUsage(const char *name);


	int parseInital(int argc, char **argv);

	int parseFilepath(int argc, char **argv);

	int parseMethodSmoothing(int argc, char **argv);

	//optional arguments
	void parseSavingPCD(int argc, char **argv);

	void parseVisualisation(int argc, char **argv);

	void parseFitlering(int argc, char **argv);



	int nr_from_file(string filepath);
	int writeToPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number);
	int writeToPCDBinary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number);

};

