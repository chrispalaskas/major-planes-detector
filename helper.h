#ifndef _HELPER_H_
#define _HELPER_H_

#include <string>
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

///
/// Helper class, contains all necessary methods
///
class Helper
{
	friend class HelperTest;
public:
	~Helper();
	void setOutLog(std::string &filename);
	void writeLog(std::stringstream &line);
	bool is_integer(const std::string& s);
	void extractMajorPlanesFromPointCloud(std::string &inputPath, int totalPlanes, std::ofstream &outfilePlanes, std::ofstream &outfileCloudWPlanes);
private:
	std::ofstream outlog;
	pcl::PointCloud<pcl::PointXYZ>::Ptr readCloudFromFile(std::string inputPath);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz2CloudConverter(std::string filePath);
	void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	void writePointsWithPlaneToFile(std::ofstream& file, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int plane);
};


#endif // _HELPER_H_