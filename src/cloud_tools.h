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

const int DIM = 3; 

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

#endif
