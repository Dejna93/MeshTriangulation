#ifndef PCL_IO_PCD_IO_H_
#define PCL_IO_PCD_IO_H_

#include <pcl/point_cloud.h>
#include <pcl/io/file_io.h>

namespace pcl
{
	class PCL_EXPORTS TxtReader : public FileReader
	{
	public:
		// Empty constructor
		TxtReader() : FileReader() {};
		// Empty deconstructor
		~TxtReader() {}

		int
			readHeader(const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0);


		/** \brief Read a point cloud data from any TXT file, and convert it to the given template format.
		* \param[in] file_name the name of the file containing the actual PointCloud data
		* \param[out] cloud the resultant PointCloud message read from disk
		*
		* \return
		*  * < 0 (-1) on error
		*  * == 0 on success
		*/
		int
			read(const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0);

		template<typename PointT> 
		int read(const std::string &file_name, pcl::PointCloud<PointT> &cloud,
			const int offset = 0) {

			pcl::PCLPointCloud2 blob;

			int res = read(file_name, blob)
		}




	};


}

#endif  //#ifndef PCL_IO_PCD_IO_H_