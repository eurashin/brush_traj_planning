#ifndef CLOUD_CONVERSIONS
#define CLOUD_CONVERSIONS

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/search.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/impl/point_types.hpp>


#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std; 

const int DIM = 3; 

void transform_pointcloud(pcl::PointCloud<pcl::PointNormal> & cloud, pcl::PointCloud<pcl::PointNormal> & transformed_cloud) {
    // Convert to message type
    sensor_msgs::PointCloud2 cloud_msg; 
    pcl::toROSMsg(cloud, cloud_msg);

    // Get the tf from /depth_optical_frame to /map
    tf::TransformListener tf_listener;
    sensor_msgs::PointCloud2 cloud_out;

    tf::StampedTransform transform_stamped;
    try {
        tf_listener.waitForTransform("/map", "/camera_depth_optical_frame", ros::Time::now(), ros::Duration(1.0));
        tf_listener.lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform_stamped);
        pcl_ros::transformPointCloud(cloud, transformed_cloud, transform_stamped);
        
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

void transform_pointcloud(pcl::PointCloud<pcl::PointXYZ> & cloud, pcl::PointCloud<pcl::PointXYZ> & transformed_cloud) {
    // Convert to message type
    sensor_msgs::PointCloud2 cloud_msg; 
    pcl::toROSMsg(cloud, cloud_msg);

    // Get the tf from /depth_optical_frame to /map
    tf::TransformListener tf_listener;
    sensor_msgs::PointCloud2 cloud_out;

    tf::StampedTransform transform_stamped;
    try {
        tf_listener.waitForTransform("/map", "/camera_depth_optical_frame", ros::Time::now(), ros::Duration(1.0));
        tf_listener.lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform_stamped);
        pcl_ros::transformPointCloud(cloud, transformed_cloud, transform_stamped);
        
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

void filter(const sensor_msgs::PointCloud2ConstPtr & cloud_msg, pcl::PointCloud<pcl::PointXYZ> & cloud) {
    pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloud2_ptr(cloud2);

    pcl_conversions::toPCL(*cloud_msg, *cloud2); 
  
    // Preprocess the cloud by...
    // ...removing distant points
    const float depth_limit = 1.0;
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud (cloud2_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, depth_limit);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
    pass.filter (*cloud2);
    
    // ... and downsampling the point cloud
    pcl::PCLPointCloud2 cloud_filtered; 
    const float voxel_grid_size = 0.005f;
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox_grid;
    vox_grid.setInputCloud (cloud2_ptr);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter (cloud_filtered);

    
    pcl::fromPCLPointCloud2(cloud_filtered,cloud);     
}


void flipCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
    for(int i=0; i<cloud->points.size(); i++) {
        cloud->points.at(i).y = cloud->points.at(i).y * -1;
        cloud->points.at(i).z = cloud->points.at(i).z * -1;
    }
}


void flipMat(cv::Mat &coords) {
    for(int i=0;i<coords.rows;i++)
    {
       pcl::PointXYZ point;
       coords.at<float>(i, 1) = coords.at<float>(i, 1) * -1;
       coords.at<float>(i, 2) = coords.at<float>(i, 2) * -1;
    }
}

cv::Mat cloudToMat(const pcl::PointCloud<pcl::PointXYZ> & cloud) {
    cv::Mat coords(cloud.points.size(), 3,  CV_64FC1);

    for(int i=0; i<cloud.points.size(); i++) {
        coords.at<float>(i, 0) = cloud.points.at(i).x;
        coords.at<float>(i, 1) = cloud.points.at(i).y;
        coords.at<float>(i, 2) = cloud.points.at(i).z;
    }
    return(coords);
}

cv::Mat cloudToMat(const pcl::PointCloud<pcl::PointXYZRGB> & cloud) {
    cv::Mat coords(cloud.points.size(), 3,  CV_64FC1);

    for(int i=0; i<cloud.points.size(); i++) {
        coords.at<float>(i, 0) = cloud.points.at(i).x;
        coords.at<float>(i, 1) = cloud.points.at(i).y;
        coords.at<float>(i, 2) = cloud.points.at(i).z;
    }
    return(coords);
}


void matToCloud(const cv::Mat coords, pcl::PointCloud<pcl::PointXYZ>::Ptr & point_cloud_ptr, std::string frame_id) {
    //char pr=100, pg=100, pb=100;

    for(int i=0;i<coords.rows;i++)
    {
       pcl::PointXYZ point;
       point.x = coords.at<float>(i, 0);
       point.y = coords.at<float>(i, 1);
       point.z = coords.at<float>(i, 2);

       point_cloud_ptr -> points.push_back(point);
    }

    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    pcl_conversions::toPCL(ros::Time::now(), point_cloud_ptr->header.stamp);
    point_cloud_ptr->header.frame_id = frame_id;

//    pcl_conversions::toPCL(ros_msg->header, point_cloud_ptr->header);
}

void color_segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, std::vector<pcl::PointIndices> & clusters) {
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);

    reg.extract (clusters);
}

void to_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, string filename, bool vtk) {
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  if(vtk)
      pcl::io::saveVTKFile (filename, triangles);
  else 
      pcl::io::savePLYFile(filename, triangles); 
}

// Segment the point cloud based on color
// Input: 
//   1) XYZRGB 
double dist(double x1, double y1, double x2, double y2) {
    return(sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)));
}

// Finds the x, y coordinate that is n percent up the point cloud and m percent along it
void central_point(const cv::Mat cloud, double n, double m) {
    // Extract all x, y values
    cv::Mat yvals = cloud.colRange(1, 2).rowRange(0, cloud.rows);
    cv::Mat xvals = cloud.colRange(0, 1).rowRange(0, cloud.rows);

    // Find the range across the x and y axis
    double xmin, xmax, ymin, ymax;
    cv::minMaxLoc(yvals, &ymin, &ymax);
    double y_range = ymax - ymin; 

    cv::minMaxLoc(xvals, &xmin, &xmax);
    double x_range = xmax - xmin;

    // Find the x-y coordinate that our point should near
    double y = ymin + (y_range * n); 
    double x = xmin + (x_range * m);

    // Find the actual point that is closest to chosen x-y coord
    double min_dist = dist(xvals.at<double>(0, 0), yvals.at<double>(1, 0), x, y); 
    int min_index = 0; 
    for(int i=1; i<cloud.rows; i++) {
       double new_dist = dist(xvals.at<double>(i, 0), yvals.at<double>(i, 0), x, y);
       cout << xvals.at<double>(i, 0) << "  " <<  yvals.at<double>(i, 0) << "  : " << new_dist << endl; 
       if(new_dist < min_dist) {
            min_dist = new_dist; 
            min_index = i; 
       }
    }

    cout << endl << "Min distance: " << min_dist << endl; 
    cout << "Point: " << cloud.at<double>(min_index, 0) << cloud.at<double>(min_index, 1) << cloud.at<double>(min_index, 2) <<  endl;   

}




// Extracts a subcloud from a point cloud
// Input: 
//   1) XYZRGB Point Cloud
//   2) Point indices
//   3) Output XYZRGB Point Cloud pointer
void extract_cloud_segment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, 
        pcl::PointIndices::Ptr indices,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & subcloud) {

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (indices);
    extract.setNegative (false);
    extract.filter (*subcloud);   
}

#endif
