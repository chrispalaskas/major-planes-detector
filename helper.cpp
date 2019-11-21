#include "helper.h"

///
/// Destructor of class, making sure we close the file stream.
///
Helper::~Helper()
{
	outlog.close();
}

///
/// Returns true if a string can be converted to an integer, else returns false.
///
bool Helper::is_integer(const std::string& s)
{
	std::string::const_iterator it = s.begin();
	while (it != s.end() && std::isdigit(*it)) ++it;
	return !s.empty() && it == s.end();
}

///
/// Sets the logger file available to class members.
///
void Helper::setOutLog(std::string &filename)
{
	outlog.open(filename);
}

///
/// Logs messages to a file and to the std output.
///
void Helper::writeLog(std::stringstream& line)
{
	outlog << line.str() << std::endl;
	std::cout << line.str() << std::endl;
	line.str(std::string());

}

///
/// Tries to open an .xyz file or a .pcd file and returns a Pointer to a pcl::PointCloud object.
///
pcl::PointCloud<pcl::PointXYZ>::Ptr Helper::readCloudFromFile(std::string inputPath)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (0 == inputPath.compare(inputPath.length() - 3, 3, "xyz"))
	{
		cloud = xyz2CloudConverter(inputPath);
	}
	else if (0 == inputPath.compare(inputPath.length() - 3, 3, "pcd"))
	{
		pcl::PCDReader reader; ///< Standard .pcd file extension reader
		reader.read(inputPath, *cloud);

	}
	else
	{
		writeLog(std::stringstream("File input not supported. Please input .xyz or .pcd file."));
		writeLog(std::stringstream(inputPath));
	}
	std::stringstream ss;
	ss << cloud->points.size() << " total points in the cloud.";
	writeLog(ss);
	return cloud;
}

///
/// Tries to read an .xyz file, given that it follows the format x y z on every line.
///
pcl::PointCloud<pcl::PointXYZ>::Ptr Helper::xyz2CloudConverter(std::string filePath)
{
	std::stringstream ss;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::ifstream infile;
	infile.open(filePath);
	/// Checks if file exists
	if (infile.fail())
	{
		ss << "File " << filePath << " not found.";
		writeLog(ss);		
		return cloud;
	}

	std::string line;
	ss << "Reading input file...";
	writeLog(ss);
	while (std::getline(infile, line))
	{
		std::istringstream iss(line);
		double x, y, z;
		/// Checks if each line is in the x y z format
		if (!(iss >> x >> y >> z)) {
			ss << "Input file is not consistent to the form {x y z}.";
			writeLog(ss);
			break;
		}
		cloud->push_back(pcl::PointXYZ(x, y, z));
	}
	return cloud;
}

///
/// Visualizes the point cloud using the PCLVisualizer. Mouse scrolls in/out, shift, control and alt freeze each axis to rotate angle.
///
void Helper::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string windowName)
{
	/// PointCloud visualization
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(windowName));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Point Cloud");
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

///
/// Writes points x, y, z and plane_id for each point to an open file stream.
///
void Helper::writePointsWithPlaneToFile(std::ofstream& file, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int plane)
{
	for (int i = 0; i < cloud->points.size(); i++)
	{
		file << cloud->at(i).x << ", " << cloud->at(i).y << ", " << cloud->at(i).z << ", " << plane << "\n";
	}
	std::stringstream ss;
	ss << "PointCloud representing the planar component " << plane << ": " << cloud->width << "*" << cloud->height <<
		" = " << cloud->points.size() << " data points.";
	writeLog(ss);
}

///
/// Opens a point cloud file and extracts #totalPlanes major planes from it. Each plane is assigned an id starting with 1,
/// and each point that belongs to it is given that id.
/// The remaining points that don't belong to the first #totalPlanes planes are given id -1.
///
void Helper::extractMajorPlanesFromPointCloud(std::string& inputPath, int totalPlanes, std::ofstream& outfilePlanes, std::ofstream& outfileCloudWPlanes, double distanceThres, bool visualize)
{
	/// Creates pointers to PointCloud objects. cloud is the total cloud, cloud_p is the extracted plane and cloud_f the remaining points.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	/// Fills in the cloud data.
	cloud = readCloudFromFile(inputPath);
	std::stringstream ss;
	if (cloud->size() == 0)
	{
		ss << "Point cloud file empty or invalid.";
		writeLog(ss);
		return;
	}
	if (visualize)
		visualizePointCloud(cloud, "Total Cloud");
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); //! coefficients: The coefficients of the plane ax+by+cz+d=0.
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); //! inliers: The indices of the inliers, i.e. the points in the plane.
	/// Creates the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZ> seg; //! seg: The segmentation object.
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE); //! We are trying to find a plane so we are using the SACMODEL_PLANE.
	seg.setMethodType(pcl::SAC_RANSAC); //! We are using the Random sample consensus method to detect points in the same plane.
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(distanceThres); //! A distance threshold has to be set depending on the dataset, unless we normalize the points.

	
	pcl::ExtractIndices<pcl::PointXYZ> extract; //! extract: Creates the filtering object.

	int i = 1, nr_points = (int)cloud->points.size();
	/// Loops while the desired number of planes has not been extracted yet.
	while (i <= totalPlanes)
	{
		/// Segments the largest planar component from the remaining cloud.
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			ss << "Could not estimate a planar model for the given dataset.";
			writeLog(ss);
			break;
		}

		/// Extracts the inliers.
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		
		/// Writes the points of the detected plane and the plane_id to a file.
		writePointsWithPlaneToFile(outfileCloudWPlanes, cloud_p, i);

		Eigen::Vector4f centroid; //! centroid: Holds the centroid of the plane.
		/// Computes the centroid point of the cloud, i.e. the plane.
		unsigned int check = pcl::compute3DCentroid(*cloud_p, centroid); 
		/// Writes the detected plane's centroid and normal coefficients to a file.
		outfilePlanes << i << ": " << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ", "
			<< coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << "\n";

		if (visualize)
		{
			pcl::PointXYZ pnt_on_line;
			double LINE_LENGTH = 50.0;
			double DISTANCE_INCREMENT = 0.01;
			for (double distfromstart = 0.0; distfromstart < LINE_LENGTH; distfromstart += DISTANCE_INCREMENT) {
				pnt_on_line.x = centroid[0] + distfromstart * coefficients->values[0];
				pnt_on_line.y = centroid[1] + distfromstart * coefficients->values[1];
				pnt_on_line.z = centroid[2] + distfromstart * coefficients->values[2];
				cloud_p->points.push_back(pnt_on_line);
			}
			std::stringstream windowNameSS;
			windowNameSS << "Plane " << i;
			visualizePointCloud(cloud_p, windowNameSS.str());
		}

		/// Creates the filtering object.
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud.swap(cloud_f);
		i++;
	}
	/// Writes the remaining points to a file.
	if (visualize)
		visualizePointCloud(cloud, "Remaining Points");
	writePointsWithPlaneToFile(outfileCloudWPlanes, cloud, -1);
}