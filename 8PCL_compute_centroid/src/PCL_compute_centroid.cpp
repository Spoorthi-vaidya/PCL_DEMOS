#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

#include <iostream>

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Object to store the centroid coordinates.
	Eigen::Vector4f centroid;
	
	pcl::compute3DCentroid(*cloud, centroid);

	std::cout << "The XYZ coordinates of the centroid are: ("
			  << centroid[0] << ", "
			  << centroid[1] << ", "
			  << centroid[2] << ")." << std::endl;
}
