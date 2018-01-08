#include <pcl/io/pcd_io.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudC(new pcl::PointCloud<pcl::PointXYZ>);

	// Read two PCD files from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloudA) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloudB) != 0)
	{
		return -1;
	}

	// Create cloud "C", with the points of both "A" and "B".
	*cloudC = (*cloudA) + (*cloudB);
	// Now cloudC->points.size() equals cloudA->points.size() + cloudB->points.size().
	pcl::io::savePCDFileASCII("output_ascii.pcd", *cloudC);
}
