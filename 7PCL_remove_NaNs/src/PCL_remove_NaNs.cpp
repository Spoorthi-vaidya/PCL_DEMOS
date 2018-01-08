#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <iostream>

int
main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::cout << "\tUsage: " << argv[0] << " <input cloud> <output cloud>" << std::endl;

		return -1;
	}

	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// The mapping tells you to what points of the old cloud the new ones correspond,
	// but we will not use it.
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

	// Save it back.
	pcl::io::savePCDFileASCII(argv[2], *cloud);
}
