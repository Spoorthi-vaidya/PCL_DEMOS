#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <vector>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExtracted(new pcl::PointCloud<pcl::PointXYZ>);//keep the extracted cluster separately..

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloudAll) != 0)
	{
		return -1;
	}

	// Plane segmentation (do not worry, we will see this later).
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);/*get plane segment from given object*/
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setOptimizeCoefficients(true);
	segmentation.setModelType(pcl::SACMODEL_PLANE);//can be changed(model)
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.1); //0.01
	segmentation.setInputCloud(cloudAll);//it gives plane

	// Object for storing the indices.
	pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);

	segmentation.segment(*pointIndices, *coefficients);// for each cloud of pointd assigns indecies..stored in point indices

	// Object for extracting points from a list of indices.
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloudAll);
	extract.setIndices(pointIndices);
	// We will extract the points that are NOT indexed (the ones that are not in a plane).
	extract.setNegative(true);//use false....if u want to display the place

	extract.filter(*cloudExtracted);
	pcl::io::savePCDFileASCII("output_extracted.pcd", *cloudExtracted);
}
