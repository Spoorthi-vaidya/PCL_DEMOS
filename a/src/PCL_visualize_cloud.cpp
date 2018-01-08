#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


// Display points with XYZ on console
	
	
	
	// Write it back to disk under a different name.
	// Another possibility would be "savePCDFileBinary()".
	
      
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}






 for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f); //to randomly generate pts
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
	


pcl::io::savePCDFileASCII("output_ascii.pcd", *cloud);
     



}
