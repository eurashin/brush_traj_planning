/*
 * Takes a "snapshot" using the Kinect and saves the pointcloud to a file test_pcd.pcd.
 * 
 * Call: rosrun process_pcl snapshot input:=/camera/depth/points
 *
 *
 *
 */

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

ros::Publisher pub;
ros::Subscriber sub; 
std::string filename; 

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("ENTERED THE CALLBACK\n");
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  
  
  // ... and downsampling the point cloud
  pcl::PCLPointCloud2 cloud_filtered; 
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox_grid;
  vox_grid.setInputCloud (cloud_ptr);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  vox_grid.filter (cloud_filtered);
  
  // Convert to PCL data type
  pcl::PointCloud<pcl::PointXYZ>::Ptr simple_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered,*simple_cloud);
  
  pcl::io::savePCDFileASCII (filename + ".pcd", *simple_cloud);
  pcl::io::savePLYFileASCII(filename + ".ply", *simple_cloud);
  ROS_INFO("Supposedly I saved...\n");
  

  /*  
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.setCameraPosition(0, 0, 0, 0, 0, 1);
  viewer.showCloud (simple_cloud);
  */

  sub.shutdown();
//  delete simple_cloud; 
//  delete cloud;  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "process_pcl");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
   
  // Read relevant parameters
//  std::string input_stream, filename; 
//  private_nh.getParam("input", input_stream);
 

  filename = (std::string)argv[1];
     
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (input_stream, 1, cloud_cb);
  sub = nh.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("woahitsmyoutput", 1);
  
  // Spin
  ros::spin();
}
