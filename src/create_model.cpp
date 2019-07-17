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
  const float voxel_grid_size = 0.002f;
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setInputCloud (subcloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  vox_grid.filter (*hair_cloud);

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*hair_cloud, *final_cloud);

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


