/*
 * Takes a "snapshot" using the Kinect and saves the pointcloud to a .ply file.
 * 
 * Call: rosrun process_pcl snapshot [filename]
 *
 * Output: 
 * 1) [filename].ply (colored)
 * 2) [filename]_no_color.ply (uncolored pointcloud)
 *
 */

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include "cloud_tools.h"

ros::Publisher pub;
ros::Subscriber sub; 
ros::Subscriber sub_image; 
std::string filename; 

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("ENTERED THE CALLBACK wubba\n");
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);
  pcl_conversions::toPCL(*cloud_msg, *cloud);



  // Preprocess the cloud by...
  // ...removing distant points
  const float depth_limit = 1.5;
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud_ptr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
  pass.filter (*cloud);
   
  // ... and downsampling the point cloud
  pcl::PCLPointCloud2 cloud_filtered; 
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox_grid;
  vox_grid.setInputCloud (cloud_ptr);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  vox_grid.filter (cloud_filtered);
  
  // Convert to PCL data type
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr simple_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(cloud_filtered,*simple_cloud);
  


  pcl::io::savePLYFileASCII(filename + ".ply", *simple_cloud);
  ROS_INFO("Supposedly I saved...\n");
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr simple_cloud_colorless(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered,*simple_cloud_colorless);
  pcl::io::savePLYFileASCII(filename + "_no_color.ply", *simple_cloud_colorless);

  sub.shutdown();
}

void image_cb(const sensor_msgs::ImageConstPtr &image) {
    // Convert image data to opencv
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }


    // Save the image to file
    cv::imwrite(filename + ".jpg", cv_ptr->image);
    
    sub_image.shutdown(); 
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "brush_traj_planning");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
   
  // Read relevant parameters
//  std::string input_stream, filename; 
//  private_nh.getParam("input", input_stream);
 

  filename = (std::string)argv[1];
  std::cout << filename << std::endl; 

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (input_stream, 1, cloud_cb);
  sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
  sub_image = nh.subscribe<sensor_msgs::Image> ("/camera/rgb/image_rect_color", 1, image_cb);

  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("woahitsmyoutput", 1);
  
  // Spin
  ros::spin();
}
