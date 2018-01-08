#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <iostream>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudAll(new pcl::PointCloud<pcl::PointNormal>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloudPoints) != 0)
	{
		return -1;
	}

	// Compute the normals of the cloud (do not worry, we will see this later).
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;//pointNormal=pointxyz+Normal
	normalEstimation.setInputCloud(cloudPoints);
	normalEstimation.setRadiusSearch(0.05);//0.05 radius to compute the normals for every normal
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);// to find nearest point
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*cloudNormals);//creates normals for every point.

	// Concatenate the fields (PointXYZ + Normal = PointNormal).
	pcl::concatenateFields(*cloudPoints, *cloudNormals, *cloudAll);//stored in cloudall after concatenating
	
	
	// Print the data to standard output.
	for (size_t currentPoint = 0; currentPoint < cloudAll->points.size(); currentPoint++)
	{
		std::cout << "Point:" << std::endl;
		std::cout << "\tXYZ:" << cloudAll->points[currentPoint].x << " "
				  << cloudAll->points[currentPoint].y << " "
				  << cloudAll->points[currentPoint].z << std::endl;
		std::cout << "\tNormal:" << cloudAll->points[currentPoint].normal[0] << " "
				  << cloudAll->points[currentPoint].normal[1] << " "
				  << cloudAll->points[currentPoint].normal[2] << std::endl;
	}



	pcl::io::savePCDFileASCII("output_ascii.pcd", *cloudAll);//save-to write..in ascii form,save it into output_ascii.pcd from cloud.
	//pcl::io::savePCDFileBinary("output_binary.pcd", *cloud);



}
