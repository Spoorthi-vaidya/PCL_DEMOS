#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// RANSAC objects: model and algorithm.
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
	// Set the maximum allowed distance to the model.
	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();

	std::vector<int> inlierIndices;
	ransac.getInliers(inlierIndices);

	// Copy all inliers of the model to another cloud.
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inlierIndices, *inlierPoints);

	pcl::PCDWriter writer;
  	writer.write<pcl::PointXYZ> ("output_inliers.pcd", *inlierPoints, false);
}
