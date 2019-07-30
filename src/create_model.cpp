// CREATES MODEL WITH PATHS
// Inputs: 
// 1) color pointcloud .ply filename
// 2) filename for newly exported .vtk model
// 3) filename for newly exported .vtk path info
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>


#include <math.h>

#include "cloud_tools.h"
#include "plan.h"

using namespace std; 


// file of the cloud, filename of output mesh
int main (int argc, char** argv)
{
  ros::init (argc, argv, "create_model");
  ros::NodeHandle nh;

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
  vector<int> indices; 
  while(cin >> hair_index) {
      vector<int> new_indices = clusters[hair_index].indices; 
      indices.insert(indices.end(), new_indices.begin(), new_indices.end());
  }
  
  pcl::PointIndices::Ptr cloud_indices(new pcl::PointIndices);
  cloud_indices->indices = indices; 
  extract_cloud_segment(cloud, cloud_indices, subcloud);
  pcl::visualization::CloudViewer viewer ("Final subcloud"); 
  viewer.showCloud (subcloud);
  while (!viewer.wasStopped ())
  {
  }

  
  // Downsampling the point cloud for meshing
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hair_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setInputCloud (subcloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  vox_grid.filter (*hair_cloud);

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*hair_cloud, *final_cloud);
  
  // Export to mesh for IK
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  transform_pointcloud(*final_cloud, *transformed_cloud);
  char modelname[80];
  strcpy(modelname, argv[2]); 
  strcat(modelname, "_mesh.ply");
  to_mesh(transformed_cloud, modelname, false);


  // Save the mesh file
  flipCloud(final_cloud); 
  char hairmeshname[80];
  strcpy(hairmeshname, argv[2]); 
  strcat(hairmeshname, ".vtk");
  to_mesh(final_cloud, hairmeshname, true);
  

  // Plan paths from our given mesh model
  a_star_path(hairmeshname, argv[3]);  


  return (0);
}


