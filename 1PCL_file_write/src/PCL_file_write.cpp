#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//storing point cloud..creates one object named cloud
//	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)//load or read file is loadpcdfile,loaded into cloud.cload is of type pointxyz
	{
		return -1;
	}

	// Display points with XYZ on console
	for (size_t i = 0; i < cloud->points.size(); ++i)//size_t :-generic datatype.
		std::cout << "\t" << cloud->points[i].x/* x y z coordinates*/
			  << " " << cloud->points[i].y
			  << " " << cloud->points[i].z
			  << std::endl;
	/*
	for (size_t i = 0; i < cloud.points.size(); ++i)
		std::cout << "\t" << cloud.points[i].x
			  << " " << cloud.points[i].y
			  << " " << cloud.points[i].z
			  << std::endl;
	*/
	
	// Write it back to disk under a different name.
	// Another possibility would be "savePCDFileBinary()".
	pcl::io::savePCDFileASCII("output_ascii.pcd", *cloud);//save-to write..in ascii form,save it into output_ascii.pcd from cloud.
	pcl::io::savePCDFileBinary("output_binary.pcd", *cloud);
//	pcl::io::savePCDFileASCII("output_ascii.pcd", cloud);
//	pcl::io::savePCDFileBinary("output_binary.pcd", cloud);

}
