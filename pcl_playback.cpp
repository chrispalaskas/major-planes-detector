/**
 * INPUT:
 *
 * 1) point cloud file folder, with .xyz extension x y z per line, or .pcd files. 
 *
 * OUTPUT: 
 *
 * 1) A "video" of the point clouds
 *
 *
 */

#include <chrono>
#include "helper.h"
#include <filesystem>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#include <pcl/filters/voxel_grid.h>

class HelperTest
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr readCloudFromFile(std::string inputPath) {
		return helper.readCloudFromFile(inputPath);
	}
	void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string windowName)
	{
		/// PointCloud visualization
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(windowName));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Point Cloud");
		viewer->initCameraParameters();
		//viewer->addCoordinateSystem();

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}
private:
	Helper helper;

};


 ///
/// Parses the arguments of the command line executable call
///
void argparser(int argc, char** argv, Helper& helper, std::string& inputFolder)
{
	std::stringstream ss;
	//! Exit if the 2 mandatory arguments are not given.
	if (argc < 2)
	{
		ss << "Please use input folder name as argument." << std::endl;
		helper.writeLog(ss);
		exit(-1);
	}

	//! The first argument should be the path of the input folder. 
	inputFolder = argv[1]; //! inputFolder: The path to the point cloud input folder.
}

void downsample_cloud(std::string folder)
{
	for (const auto& entry : std::filesystem::directory_iterator(folder))
	{
		const wchar_t* c = entry.path().native().c_str();
		std::wstring ws(c);
		std::string filename(ws.begin(), ws.end());

		pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
		pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

		// Fill in the cloud data
		pcl::PCDReader reader;
		// Replace the path below with the path where you saved your file
		reader.read(filename, *cloud); // Remember to download the file first!

		std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
			<< " data points (" << pcl::getFieldsList(*cloud) << ").";

		// Create the filtering object
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.1f, 0.1f, 0.1f);
		sor.filter(*cloud_filtered);

		std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
			<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

		pcl::PCDWriter writer;
		filename = filename.substr(0, filename.size() - 4) + "_downsampled.pcd";
		writer.write(filename, *cloud_filtered,
			Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
	}
}

void visualize_pcl_folder(std::string folder)
{
	HelperTest helper;
	std::vector<std::string> filesV;
	for (const auto& entry : std::filesystem::directory_iterator(folder))
	{
		const wchar_t* c = entry.path().native().c_str();
		std::wstring ws(c);
		std::string str(ws.begin(), ws.end());
		filesV.push_back(str);
	}

	if (filesV.empty())
	{
		std::cout << "No files found in folder." << std::endl;
		return;
	}
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = helper.readCloudFromFile(filesV[0]);
	Eigen::Vector4f centroid; //! centroid: Holds the centroid of the plane.
	/// Computes the centroid point of the cloud, i.e. the plane.
	unsigned int check = pcl::compute3DCentroid(*cloud, centroid);
	/// PointCloud visualization
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Player"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Point Cloud");
	viewer->initCameraParameters();
	viewer->setCameraPosition(30,0,30, 1, 0, 1);
	std::vector<pcl::visualization::Camera> cams;
	viewer->getCameras(cams);

	for (auto&& camera : cams)
	{
		camera.focal[0] = centroid[0];
		camera.focal[1] = centroid[1];
		camera.focal[2] = centroid[2];
	}

	viewer->setCameraParameters(cams[0]);
	viewer->getRendererCollection()->GetFirstRenderer()->GetActiveCamera()->Azimuth(45);
	//viewer->getRendererCollection()->GetFirstRenderer()->GetActiveCamera()->Yaw(180);

	//viewer->addCoordinateSystem();

	int nextIndex = 0;
	while (nextIndex<filesV.size())
	{
		viewer->spinOnce(100);
		viewer->removeAllShapes();
		viewer->removeAllPointClouds();
		nextIndex++;
		cloud = helper.readCloudFromFile(filesV[nextIndex]);
		
		viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Point Cloud");
		viewer->updatePointCloud(cloud);

		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	
}

int main (int argc, char** argv)
{
	auto start = std::chrono::system_clock::now(); //! start: Holds the start of the program's runtime.
	Helper helper; //! helper: Helper object, to access the class' methods.
	std::string logfile = "log_playback.txt"; //! logfile: The output filename of all the logging.
	helper.setOutLog(logfile); //! Initialize the logging file.

	std::string inputFolder="";
	//! Parse the input arguments of the execution call.
	argparser(argc, argv, helper, inputFolder);
	//downsample_cloud(inputFolder);
	//exit(0);
	visualize_pcl_folder(inputFolder);

	std::cout << inputFolder << std::endl;
	
	auto end = std::chrono::system_clock::now(); //! end: Holds the end of the program's runtime.
	std::chrono::duration<double> elapsed_seconds = end - start; //! elapsed_seconds: Holds the duration of the runtime in seconds.
	std::stringstream ss; //! ss: Stringstream that will carry the message to be logged.
	ss << "Elapsed time: " << elapsed_seconds.count() << "s";
	helper.writeLog(ss);
	return (0);
}

