#ifndef IO_H
#define IO_H

#include <string>
#include <regex>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <iterator>
#include <iostream>
#include <sstream>

#include <pcl/io/vtk_lib_io.h>
#include <vector>
#include "include/Dao.h"

using namespace std;
namespace filesys = boost::filesystem;



class Io
{
public:
    Io();

	int input(int argc, char **argv);

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCD();


	// SETTERS and GETTERS
	int setFilepath(std::string filepath);

    void setParams(std::string filepath);

    Dao getDao();

    void setPointFolder(std::string folder);

    void setSTLFolder(std::string folder);


	boost::filesystem::path getFilepath();


    boost::filesystem::path getStlFolder();


    boost::filesystem::path createFolder(std::string filepath);

    boost::filesystem::path createFolder(boost::filesystem::path filepath);

    void printStack();

    int saveSTL(pcl::PolygonMesh output_mesh);

    int saveSTL(int index, pcl::PolygonMesh output_mesh);

    int savePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    int savePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index);

    // int writeToPCDBinary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number);


    //dont imlpement
private:

    Dao dao;
	boost::filesystem::path filepath;
    boost::filesystem::path current_folder;


    boost::filesystem::path point_folder;
    boost::filesystem::path stl_folder;


    void setPointFolder(boost::filesystem::path filepath);

    void setSTLFolder(boost::filesystem::path filepath);

	void printUsage(const char *name);


	int parseInital(int argc, char **argv);

	int parseFilepath(int argc, char **argv);

    int parseParams(int argc, char **argv);

    filesys::path makePathToNewPoint(std::string filepath);

    filesys::path makePathToNewSTL(std::string filepath);

    std::string join(std::string name, int id);

    std::string join(filesys::path name, int id);


    int id_from_file(boost::filesystem::path filepath);


    int create_folder(boost::filesystem::path path);

    int isFolderByName(std::string name, boost::filesystem::path folder);


};

#endif //MESH_TRIANGULATION_POISSONDAO_H

