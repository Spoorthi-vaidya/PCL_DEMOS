#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr concaveHull(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Get the plane model, if present.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setOptimizeCoefficients(true);
	pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
	segmentation.segment(*inlierIndices, *coefficients);

	if (inlierIndices->indices.size() == 0)
		std::cout << "Could not find a plane in the scene." << std::endl;
	else
	{
		// Copy the points of the plane to a new cloud.
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inlierIndices);
		extract.filter(*plane);

		// Object for retrieving the concave hull.
		pcl::ConcaveHull<pcl::PointXYZ> hull;
		hull.setInputCloud(plane);
		// Set alpha, which is the maximum length from a vertex to the center of the voronoi cell
		// (the smaller, the greater the resolution of the hull).
		hull.setAlpha(0.1);
		hull.reconstruct(*concaveHull);

		// Visualize the hull.
		pcl::visualization::CloudViewer viewerPlane("Concave hull");
		viewerPlane.showCloud(concaveHull);
		while (!viewerPlane.wasStopped())
		{
			// Do nothing but wait.
		}
	}
}
