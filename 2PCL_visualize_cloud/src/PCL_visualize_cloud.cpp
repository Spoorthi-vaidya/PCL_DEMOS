#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

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

	// Cloud viewer object. You can set the window title.
	pcl::visualization::CloudViewer viewer(argv[1]);
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
		// Do nothing but wait.
	}
}
