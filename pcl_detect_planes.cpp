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

 ///
/// Parses the arguments of the command line executable call
///
void argparser(int argc, char** argv, Helper& helper, int& totalPlanes, std::string& inputPath, double& distanceThres, bool& visualizePC)
{
	std::stringstream ss;
	std::string enabledVisualization = "Visualization of Point Cloud enabled.";
	std::string disabledVisualization = "Visualization of Point Cloud disabled.";
	std::string optionalThirdArg = "The third argument is optional, but if set, it has to be ";
	std::string optionalFourthArg = "The fourth argument is optional, but if set, it has to be ";
	std::string doubleForThres = "a double for the point distance threshold, with default = 1.0.";
	std::string boolForVisual = "a boolean true / false for visualization, with default false.";
	//! Exit if the 2 mandatory arguments are not given.
	if (argc < 3)
	{
		ss << "Please use input file name and number of planes as arguments." << std::endl
			<< "Optional 1: distance threshold for plane detection. Default: 1.0" << std::endl
			<< "Optional 2: enable visualization. Default: false" << std::endl
			<< "E.g. <pcl_detect_planes.exe point_cloud.txt 3 0.1 true>";
		helper.writeLog(ss);
		exit(-1);
	}

	//! The first argument should be the path of the input file. If it is not a valid cloud file it will fail later.
	inputPath = argv[1]; //! inputPath: The path to the point cloud input file.
	if (!helper.is_integer(argv[2])) //! Verifying that the second argument is an integer.
	{
		ss << "The second argument has to be an integer.";
		helper.writeLog(ss);
		exit(-1);
	}
	totalPlanes = atoi(argv[2]); //! totalPlanes: The number of planes that should be detected.

	//! If three arguments were given, the third and optional could be the distance threshold or the visualization flag.
	if (argc == 4)
	{
		if (std::string(argv[3]) == "true" || std::string(argv[3]) == "false")
		{
			visualizePC = std::string(argv[3]) == "true";
			if (visualizePC)
				ss << enabledVisualization;
			else
				ss << disabledVisualization;
			helper.writeLog(ss);
		}
		else if (atof(argv[3]) == 0.0)
		{
			ss << optionalThirdArg << doubleForThres << std::endl
				<< "OR " << boolForVisual;
			helper.writeLog(ss);
			exit(-1);
		}
		else
			distanceThres = atof(argv[3]);
	}
	//! If 4 arguments are given the third should be the distance threshold and the fourth the visualization flag.
	else if (argc == 5)
	{
		if (atof(argv[3]) == 0.0)
		{
			ss << optionalThirdArg << doubleForThres;
			helper.writeLog(ss);
			exit(-1);
		}
		else
			distanceThres = atof(argv[3]);

		if (std::string(argv[4]) == "true" || std::string(argv[4]) == "false")
		{
			visualizePC = std::string(argv[4]) == "true";
			if (visualizePC)
				ss << enabledVisualization;
			else
				ss << disabledVisualization;
			helper.writeLog(ss);
		}
		else
		{
			ss << optionalFourthArg << boolForVisual;
			helper.writeLog(ss);
			exit(-1);
		}
	}
}

int main (int argc, char** argv)
{
	auto start = std::chrono::system_clock::now(); //! start: Holds the start of the program's runtime.
	Helper helper; //! helper: Helper object, to access the class' methods.
	std::string logfile = "log.txt"; //! logfile: The output filename of all the logging.
	helper.setOutLog(logfile); //! Initialize the logging file.

	int totalPlanes=0;
	std::string inputPath="";
	double distanceThres = 1.0;
	bool visualizePC = false;
	//! Parse the input arguments of the execution call.
	argparser(argc, argv, helper, totalPlanes, inputPath, distanceThres, visualizePC);
	
	
	std::string outputPlanes = "Detected_Planes.txt"; //! outputPlanes: The output filename with the detected planes, centroid and normal.
	std::string outputPointCloudWithPlaneIDs = "Point_Cloud_with_Plane_id.txt"; //! outputPointCloudWithPlaneIDs: The output filename with the cloud points and plane_id.

	std::ofstream outfilePlanes;	
	std::ofstream outfileCloudWPlanes;
	outfileCloudWPlanes.open(outputPointCloudWithPlaneIDs);
	outfilePlanes.open(outputPlanes);

	/// Calls extractMajorPlanesFromPointCloud, where all processing takes place.
	helper.extractMajorPlanesFromPointCloud(inputPath, totalPlanes, outfilePlanes, outfileCloudWPlanes, distanceThres, visualizePC);
	
	outfilePlanes.close();
	outfileCloudWPlanes.close();
	auto end = std::chrono::system_clock::now(); //! end: Holds the end of the program's runtime.
	std::chrono::duration<double> elapsed_seconds = end - start; //! elapsed_seconds: Holds the duration of the runtime in seconds.
	std::stringstream ss; //! ss: Stringstream that will carry the message to be logged.
	ss << "Elapsed time: " << elapsed_seconds.count() << "s";
	helper.writeLog(ss);
	return (0);
}

