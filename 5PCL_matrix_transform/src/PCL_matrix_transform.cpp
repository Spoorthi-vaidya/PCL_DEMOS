//expeditions



#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Transformation matrix object, initialized to the identity matrix
	// (a null transformation).
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();//affine3f can also b used..identity matrix is created.

	// Set a rotation around the Z axis.
	float theta = M_PI / 2; // 180 degrees.
	transformation(0, 0) = cos(theta);
	transformation(0, 1) = -sin(theta);
	transformation(1, 0) = sin(theta);
	transformation(1, 1) = cos(theta);

	// Set a translation on the X axis.
	transformation(0, 3) = 1.0f; // 1 meter (positive direction).
	
	//Print the transformation
	std::cout << transformation << std::endl;	

	pcl::transformPointCloud(*cloud, *transformed, transformation);//transformPointCloud(input,output,transform matrix);

	// Visualize both the original and the result.
	pcl::visualization::PCLVisualizer viewer(argv[1]);//can also use cloudViewer
	viewer.addPointCloud(cloud, "original");//adds point into viewer
	// The transformed one's points will be red in color.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(transformed, 0,255, 0);//green color
	viewer.addPointCloud(transformed, colorHandler, "transformed");
	// Add 3D colored axes to help see the transformation.
	viewer.addCoordinateSystem(1.0, "reference", 0);//show the axes

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
pcl::io::savePCDFileASCII("output_ascii.pcd", *transformed);
}
