#pragma once
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//TEMP STRUCT
struct PointXYZDEBUG {
	float x;
	float y;
	float z;
	
};
//TEMP STRUCT 
struct PointXYZS
{
	std::string x;
	std::string y;
	std::string z;
};

//  They will be remove after link with PCL main core

namespace pcl {

	std::istream& operator>> (std::istream &input, PointXYZDEBUG & point);
	std::ostream& operator<<(std::ostream & out, const PointXYZDEBUG &p);

	class TXTFile
	{
	
	public:
		TXTFile();
		~TXTFile();
		std::vector<PointXYZDEBUG> loadTxtFile(std::string &file_name );
		int loadCloud();

	private:
		std::vector<PointXYZDEBUG> points;

		int loadFile(std::string &file_name,  std::ifstream & file);
		friend std::istream& operator>> (std::istream & input, PointXYZDEBUG & point);
		friend std::ostream& operator<<(std::ostream & out, const PointXYZDEBUG &p);
	};

	
}

