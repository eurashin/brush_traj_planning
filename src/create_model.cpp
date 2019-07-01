#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/search.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>


#include <math.h>

#include "cloud_tools.h"

using namespace std; 

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


void to_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const char* filename) {
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

  pcl::io::saveVTKFile (filename, triangles);
}

// Segment the point cloud based on color
// Input: 
//   1) XYZRGB 
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

// file of the cloud, filename of output mesh
int main (int argc, char** argv)
{

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  if ( pcl::io::loadPLYFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  // Find color segmented clusters
  std::vector<pcl::PointIndices> clusters; 
  color_segmentation(cloud, clusters);
 
  // Iterate through each cluster and display
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr subcloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  for(int i=0; i<clusters.size(); i++) {
      std::cout << "viewing..." << std::endl;
      pcl::PointIndices::Ptr indices(new pcl::PointIndices(clusters[i]));
      extract_cloud_segment(cloud, indices, subcloud);

      pcl::visualization::CloudViewer viewer ("Segment number " + std::to_string(i)); 
      viewer.showCloud (subcloud);
      while (!viewer.wasStopped ())
      {
      }
  }

  // Extract segment with just the hair
  std::cout << "Which segment was the hair? " << std::endl; 
  int hair_index; 
  std::cin >> hair_index;
      
  pcl::PointIndices::Ptr indices(new pcl::PointIndices(clusters[hair_index]));
  extract_cloud_segment(cloud, indices, subcloud); 
  pcl::io::savePLYFileASCII(argv[2], *subcloud);
  
  
  // Downsampling the point cloud for meshing
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hair_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  const float voxel_grid_size = 0.008f;
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setInputCloud (subcloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  vox_grid.filter (*hair_cloud);

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*hair_cloud, *final_cloud);

  // Invert the point cloud (y-axis is flipped)
  flipCloud(final_cloud); 
  to_mesh(final_cloud, argv[2]);

  return (0);
}


