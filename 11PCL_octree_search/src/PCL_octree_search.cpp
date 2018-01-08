#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>

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

	// Octree object, with a maximum resolution of 128
	// (resolution at lowest octree level).
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(128);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	// We will find all neighbors within the same voxel as this point.
	// (it does not have to be one of the cloud's, we can use any coordinate).
	pcl::PointXYZ point;
	point.x = -1;
	point.y =-1;
	point.z =-1;
	// This vector will store the output neighbors.
	std::vector<int> pointIndices;
	// Perform the search, and print out results.
	if (! octree.voxelSearch(point, pointIndices) > 0)
	{
		std::cout << "Neighbors in the same voxel:" << std::endl;
		for (size_t i = 0; i < pointIndices.size(); ++i)
			std::cout << "\t" << cloud->points[pointIndices[i]].x
					  << " " << cloud->points[pointIndices[i]].y
					  << " " << cloud->points[pointIndices[i]].z << std::endl;
	}

	// We will find the 5 nearest neighbors of the point.
	// This vector will store their squared distances to the search point.
	std::vector<float> squaredDistances;
	if (octree.nearestKSearch(point, 5, pointIndices, squaredDistances) > 0)
	{
		std::cout << "5 nearest neighbors of the point:" << std::endl;
		for (size_t i = 0; i < pointIndices.size(); ++i)
			std::cout << "\t" << cloud->points[pointIndices[i]].x
					  << " " << cloud->points[pointIndices[i]].y
					  << " " << cloud->points[pointIndices[i]].z
					  << " (squared distance: " << squaredDistances[i] << ")" << std::endl;
	}

	// Now we find all neighbors within 1cm of the point
	// (inside a sphere of radius 1cm centered at the point).
	// The point DOES have to belong in the cloud.
	if (octree.radiusSearch(point, 0.01, pointIndices, squaredDistances) > 0)
	{
		std::cout << "Neighbors within 1cm:" << std::endl;
		for (size_t i = 0; i < pointIndices.size(); ++i)
			std::cout << "\t" << cloud->points[pointIndices[i]].x
					  << " " << cloud->points[pointIndices[i]].y
					  << " " << cloud->points[pointIndices[i]].z
					  << " (squared distance: " << squaredDistances[i] << ")" << std::endl;
	}
}
