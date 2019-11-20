#include <iostream>
#include "gtest/gtest.h"
#include "helper.h"

class HelperTest : public ::testing::Test
{
protected:
	pcl::PointCloud<pcl::PointXYZ>::Ptr readCloudFromFile(std::string inputPath) {
		return helper.readCloudFromFile(inputPath); }
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz2CloudConverter(std::string filePath) {
		return helper.xyz2CloudConverter(filePath);
	}
private:
	Helper helper;
};


Helper helperPublic;
TEST(HelperPublicTest, Test_is_integer) {
	ASSERT_TRUE(helperPublic.is_integer("2"));
	ASSERT_FALSE(helperPublic.is_integer("2.2"));
	ASSERT_FALSE(helperPublic.is_integer("a string"));
}

TEST_F(HelperTest, Test_readCloudFromFile) {
	std::string inputPath = "testCloudName.xyz";
	std::ofstream outfile;
	outfile.open(inputPath);
	outfile << "1.3 2.1 -0.5\n";
	outfile.close();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readCloudFromFile(inputPath);
	double a = cloud->points[0].x;
	ASSERT_LT(0, cloud->points.size());
	EXPECT_NEAR(1.3, cloud->points[0].x, 1e-5);
	EXPECT_NEAR(2.1, cloud->points[0].y, 1e-5);
	EXPECT_NEAR(-0.5, cloud->points[0].z, 1e-5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_empty = readCloudFromFile("nofile");
	ASSERT_EQ(0, cloud_empty->points.size());
	// Remove file
	ASSERT_EQ(0, remove(inputPath.c_str()));
}

TEST_F(HelperTest, Test_xyz2CloudConverter) {
	std::string inputPath = "testCloudName.xyz";
	std::ofstream outfile;
	outfile.open(inputPath);
	outfile << "1.3 2.1 -0.5\n";
	outfile.close();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = xyz2CloudConverter(inputPath);
	double a = cloud->points[0].x;
	ASSERT_LT(0, cloud->points.size());
	EXPECT_NEAR(1.3, cloud->points[0].x, 1e-5);
	EXPECT_NEAR(2.1, cloud->points[0].y, 1e-5);
	EXPECT_NEAR(-0.5, cloud->points[0].z, 1e-5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_empty = xyz2CloudConverter("nofile");
	ASSERT_EQ(0, cloud_empty->points.size());
	// Remove file
	ASSERT_EQ(0, remove(inputPath.c_str()));
}

TEST(HelperPublicTest, Test_extractMajorPlanesFromPointCloud) {
	std::string planes = "testOutPlanes.txt";
	std::string cloudWPlanes = "testOutCloudWPlanes.txt";
	std::string cloud = "cloud.xyz";
	std::ofstream outfilePlanes(planes);
	std::ofstream outfileCloudWPlanes(cloudWPlanes);
	std::ofstream cloudfile(cloud);
	std::vector<std::vector<double>> cloudV;
	/// Create a cloud of 9 points. 5 on the z=0 plane and 4 on the z=100 plane
	cloudV.push_back(std::vector<double>({ -10, 10, 0.000001 }));
	cloudV.push_back(std::vector<double>({ 10, 10, 0.000001 }));
	cloudV.push_back(std::vector<double>({ 10, -10, 0.000001 }));
	cloudV.push_back(std::vector<double>({ -10, -10, 0.000001 }));
	cloudV.push_back(std::vector<double>({ 0, 0, 0.000001 }));
	cloudV.push_back(std::vector<double>({ 100, 100, 100 }));
	cloudV.push_back(std::vector<double>({ 95, 100, 100 }));
	cloudV.push_back(std::vector<double>({ 100, 95, 100 }));
	cloudV.push_back(std::vector<double>({ 95, 95, 100 }));

	for (auto point : cloudV)
	{
		cloudfile << point[0] << " " << point[1] << " " << point[2] << "\n";
	}

	cloudfile.close();
	/// Find the 2 major planes of the cloud.
	helperPublic.extractMajorPlanesFromPointCloud(cloud, 2, outfilePlanes, outfileCloudWPlanes);
	outfilePlanes.close();
	outfileCloudWPlanes.close();

	/// Verify that each point is classified in the right plane.
	/// We will do that using the planes coefficients in the form ax+by+cz+d=0.
	std::ifstream infilePlanes;
	infilePlanes.open(planes);
	std::vector<double> coefficients1;
	std::vector<double> coefficients2;
	std::string line;
	/// Get centroid and coefficients for each plane.
	int planeCnt = 0;
	while (std::getline(infilePlanes, line))
	{
		planeCnt++;
		std::stringstream ss(line);
		std::vector<double> coeffsV;
		std::string planestr;
		getline(ss, planestr, ':');

		while (ss.good())
		{
			std::string substr;
			getline(ss, substr, ',');
			coeffsV.push_back(std::stod(substr.c_str()));
		}

		if (planestr == "1")
			coefficients1 = coeffsV;
		else
			coefficients2 = coeffsV;
	}
	ASSERT_EQ(planeCnt, 2);
	std::ifstream infileCloudWPlanes;
	infileCloudWPlanes.open(cloudWPlanes);
	ASSERT_FALSE(infileCloudWPlanes.fail());
	std::vector<std::vector<double>> cloud1;
	std::vector<std::vector<double>> cloud2;
	/// Get points of each plane.
	int pointCnt = 0;
	while (std::getline(infileCloudWPlanes, line))
	{
		pointCnt++;
		std::stringstream ss(line);
		std::vector<double> pointV;
		while (ss.good())
		{
			std::string substr;
			getline(ss, substr, ',');
			pointV.push_back(std::stod(substr.c_str()));
		}
		if (pointV[3] == 1.0)
			cloud1.push_back(pointV);
		else
			cloud2.push_back(pointV);
	}
	ASSERT_EQ(pointCnt, 9); ///Verify we found only 2 planes.
	/// Verify each point with the plane's coefficients.
	for (auto point : cloud1)
	{
		//ax+bx+cz+d=0
		double d = coefficients1[3] * point[0] + coefficients1[4] * point[1] + coefficients1[5] * point[2];
		EXPECT_NEAR(0, d, 1e-5);
	}
	for (auto point : cloud2)
	{
		//ax+bx+cz+d=0
		double d = coefficients2[3] * point[0] + coefficients2[4] * point[1] + coefficients2[5] * point[2];
		EXPECT_NEAR(100, d, 1e-5);
	}
	ASSERT_EQ(cloud1.size(), 5);
	ASSERT_EQ(cloud2.size(), 4);
	// Remove files
	infilePlanes.close();
	infileCloudWPlanes.close();
	cloudfile.close();
	ASSERT_EQ(0, remove(planes.c_str()));
	ASSERT_EQ(0, remove(cloudWPlanes.c_str()));
	ASSERT_EQ(0, remove(cloud.c_str()));
}