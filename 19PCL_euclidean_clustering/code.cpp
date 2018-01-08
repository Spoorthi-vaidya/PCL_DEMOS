

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
//#include <ros/ros.h>
// PCL specific includes
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/cvfh.h>



#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <pcl/kdtree/kdtree.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>







typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::VFHSignature308 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.03f);
float scene_ss_ (0.05f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.015f);
float cg_thresh_ (5.0f);

pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());




int main()
{
 // Read in the scene data
  pcl::PCDReader reader;
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>), cloud_f (new pcl::PointCloud<PointType>);
  reader.read ("/home/ubuntu/catkin_ws/local global trying/milk_cartoon_all_small_clorox.pcd", *cloud);
   reader.read ("/home/ubuntu/catkin_ws/local global trying/Framescloud1.pcd", *model);
   std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

 //Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointType> vg;
  pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<PointType> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "object_" << j << ".pcd";
    writer.write<PointType> (ss.str (), *cloud_cluster, false); //*
    j++;
  }


//for(int m=0;m<j;m++)
//{
// Cloud for storing the object.
	pcl::PointCloud<PointType>::Ptr object(new pcl::PointCloud<PointType>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the CVFH descriptors.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr cluster_descriptors(new pcl::PointCloud<pcl::VFHSignature308>);


	pcl::PointCloud<pcl::Normal>::Ptr modelnormals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the CVFH descriptors.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr model_descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

	// Note: you should have performed preprocessing to cluster out the object
	// from the cloud, and save it to this individual file.

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<PointType>("/home/ubuntu/catkin_ws/local global trying/build/object_0.pcd", *object) != 0)
	{
		return -1;
	}

	// Estimate the normals.
	pcl::NormalEstimation<PointType, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(object);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// CVFH estimation object.
	pcl::CVFHEstimation<PointType, pcl::Normal, pcl::VFHSignature308> cvfh;
	cvfh.setInputCloud(object);
	cvfh.setInputNormals(normals);
	cvfh.setSearchMethod(kdtree);
	// Set the maximum allowable deviation of the normals,
	// for the region segmentation step.
	cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
	// Set the curvature threshold (maximum disparity between curvatures),
	// for the region segmentation step.
	cvfh.setCurvatureThreshold(1.0);
	// Set to true to normalize the bins of the resulting histogram,
	// using the total number of points. Note: enabling it will make CVFH
	// invariant to scale just like VFH, but the authors encourage the opposite.
	cvfh.setNormalizeBins(false);

	cvfh.compute(*cluster_descriptors);


	pcl::io::savePCDFileASCII("cluster_descriptor.pcd", *cluster_descriptors);




// Estimate the normals.
	pcl::NormalEstimation<PointType, pcl::Normal> modelnormalEstimation;
	modelnormalEstimation.setInputCloud(model);
	modelnormalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<PointType>::Ptr kdtreemodel(new pcl::search::KdTree<PointType>);
	modelnormalEstimation.setSearchMethod(kdtreemodel);
	modelnormalEstimation.compute(*modelnormals);

	// CVFH estimation object.
	pcl::CVFHEstimation<PointType, pcl::Normal, pcl::VFHSignature308> modelcvfh;
	modelcvfh.setInputCloud(model);
	modelcvfh.setInputNormals(modelnormals);
	modelcvfh.setSearchMethod(kdtreemodel);
	// Set the maximum allowable deviation of the normals,
	// for the region segmentation step.
	modelcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
	// Set the curvature threshold (maximum disparity between curvatures),
	// for the region segmentation step.
	modelcvfh.setCurvatureThreshold(1.0);
	// Set to true to normalize the bins of the resulting histogram,
	// using the total number of points. Note: enabling it will make CVFH
	// invariant to scale just like VFH, but the authors encourage the opposite.
	modelcvfh.setNormalizeBins(false);

	modelcvfh.compute(*model_descriptors);



	pcl::io::savePCDFileASCII("model_descriptor.pcd", *model_descriptors);
/*
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

 // pcl::KdTreeFLANN<DescriptorType> match_search;//KdTreeFLANN is a generic type of 3D spatial locator using kD-tree structures.

//The class is making use of the FLANN (Fast Library for Approximate Nearest Neighbor) p
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.


  for (size_t i = 0; i < cluster_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
   if (!pcl_isfinite (cluster_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (cluster_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;



*/


//}


return 0;

}

















