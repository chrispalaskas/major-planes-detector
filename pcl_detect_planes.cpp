/**
 * INPUT:
 *
 * 1) point cloud file, with .xyz extension x y z per line, or .pcd file. 
 *
 * 2) Number of planes to be detected
 *
 * OUTPUT: 
 *
 * 1) A file with the major planes detected plane_id: centroid_x, centroid_y, centroid_z, normal_x, normal_y, normal_z
 *
 * 2) A point cloud file, containing x, y, z, plane_id. -1 for points that don't belong to one of the detected planes.
 *
 *
 */
#include <chrono>
#include "helper.h"

int main (int argc, char** argv)
{
	auto start = std::chrono::system_clock::now(); //! start: Holds the start of the program's runtime.
	Helper helper; //! helper: Helper object, to access the class' methods.
	std::string logfile = "log.txt"; //! logfile: The output filename of all the logging.
	helper.setOutLog(logfile); //! Initialize the logging file.
	std::stringstream ss; //! ss: Stringstream that will carry the message to be logged.

	if (argc < 3)
	{
		ss << "Please use input file name and number of planes as arguments." << std::endl
		   << "Optional: distance threshold for plane detection. Default: 1.0" << std::endl
		   << "E.g. <pcl_detect_planes.exe point_cloud.txt 3>";
		helper.writeLog(ss);
		exit(-1);
	}

	std::string inputPath = argv[1]; //! inputPath: The path to the point cloud input file.
	if (!helper.is_integer(argv[2])) //! Verifying that the second argument is an integer.
	{
		ss << "The second argument has to be an integer.";
		helper.writeLog(ss);
		exit(-1);
	}
	int totalPlanes = atoi(argv[2]); //! totalPlanes: The number of planes that should be detected.
	double distanceThres = 1.0;
	if (argc == 4)
	{
		if (atof(argv[3]) == 0.0)
		{
			ss << "The third argument is optional, but if set has to be a double. Default is 1.0";
			helper.writeLog(ss);
			exit(-1);
		}
		else
			distanceThres = atof(argv[3]);
	}

	std::string outputPlanes = "Detected_Planes.txt"; //! outputPlanes: The output filename with the detected planes, centroid and normal.
	std::string outputPointCloudWithPlaneIDs = "Point_Cloud_with_Plane_id.txt"; //! outputPointCloudWithPlaneIDs: The output filename with the cloud points and plane_id.

	std::ofstream outfilePlanes;	
	std::ofstream outfileCloudWPlanes;
	outfileCloudWPlanes.open(outputPointCloudWithPlaneIDs);
	outfilePlanes.open(outputPlanes);

	/// Calls extractMajorPlanesFromPointCloud, where all processing takes place
	helper.extractMajorPlanesFromPointCloud(inputPath, totalPlanes, outfilePlanes, outfileCloudWPlanes, distanceThres);
	
	outfilePlanes.close();
	outfileCloudWPlanes.close();
	auto end = std::chrono::system_clock::now(); //! end: Holds the end of the program's runtime.
	std::chrono::duration<double> elapsed_seconds = end - start; //! elapsed_seconds: Holds the duration of the runtime in seconds.
	ss << "Elapsed time: " << elapsed_seconds.count() << "s";
	helper.writeLog(ss);
	return (0);
}

