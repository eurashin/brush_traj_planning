// Will listen to the /camera/depth/points topic and align an original model pointcloud to the 
// new scene. 
//
// Usage: rosrun brush_traj_planning live_registration [model.ply]
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

#include <iostream>
#include "cloud_tools.h"
#include <pcl/io/ply_io.h>
#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/surface_matching.hpp"

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

using namespace cv;
using namespace ppf_match_3d;
using namespace std; 

// Globals
ros::Publisher transform_pub;
ros::Publisher aligned_pub;
Mat model, path; 
ppf_match_3d::PPF3DDetector detector; // Model to detect face 

const double viewpoint[3] = {0,0,0};

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Transfer scene to filtered point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud; 
    filter(cloud_msg, cloud); // Remove distant points/downsample

    // Conver to OpenCV Mat format
    cv::Mat coords = cloudToMat(cloud);
    Mat scene; 
    computeNormalsPC3d(coords, scene, 6, false, viewpoint);
    
    // Match the model to the scene and get the pose
    cout << endl << "Starting matching..." << endl;
    vector<Pose3DPtr> results;
    int64 tick1 = cv::getTickCount();
   
    detector.match(scene, results, 0.025, 0.05);
    int64 tick2 = cv::getTickCount();
    cout << endl << "PPF Elapsed Time " <<
         (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;
        
    // Get only the best result
    int N = 1;
    vector<Pose3DPtr> resultsSub(results.begin(),results.begin()+N);
    // Create an instance of ICP
    ICP icp(100, 0.005f, 2.5f, 8);
    int64 t1 = cv::getTickCount();

    // Register for all selected poses
    cout << endl << "Performing ICP" << endl;
    icp.registerModelToScene(model, scene, resultsSub);
    int64 t2 = cv::getTickCount();

    cout << endl << "ICP Elapsed Time " <<
         (t2-t1)/cv::getTickFrequency() << " sec" << endl;

    // Transform model to fit scene
    Pose3DPtr result = resultsSub[0];
    tf::Vector3 position(result->t[0], result->t[1], result->t[2]);
    tf::Quaternion quat(result->q[0], result->q[1], result->q[2], result->q[3]);
    tf::Transform transform(quat, position);

    tf::TransformListener tf_listener;
    sensor_msgs::PointCloud2 cloud_out;
    tf::StampedTransform transform_stamped;
    try {
        tf_listener.waitForTransform("/map", "/camera_depth_optical_frame", ros::Time::now(), ros::Duration(1.0));
        tf_listener.lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform_stamped);    
         
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }

    tf::Transform new_transform(transform_stamped.getRotation(), transform_stamped.getOrigin());
    tf::Transform final_transform = new_transform * transform;
    geometry_msgs::Transform msg; 
    tf::transformTFToMsg(final_transform, msg);
    transform_pub.publish(msg); 

    // Broadcast topic with new alignment
    Mat aligned_model = transformPCPose(model, result->pose);
    result->printPose();
    pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZ>);
    matToCloud(aligned_model, newcloud, "camera_depth_optical_frame");
    aligned_pub.publish(*newcloud); 
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        ROS_ERROR("Not enough input arguments. Usage: rosrun brush_traj_planning live_registration [model.ply]");
        exit(1);
    }

    string modelFileName = (string)argv[1];

    // Read the model 
    Mat pc = loadPLYSimple(modelFileName.c_str(), 0);
    computeNormalsPC3d(pc, model, 6, false, viewpoint);

    // Now train the model
    cout << "Training..." << endl;
    int64 tick1 = cv::getTickCount();
    ppf_match_3d::PPF3DDetector temp_detector(0.025, 0.05); 
    temp_detector.trainModel(model);
    int64 tick2 = cv::getTickCount();
    detector = temp_detector; 
    cout << endl << "Training complete in "
         << (double)(tick2-tick1)/ cv::getTickFrequency()
         << " sec" << endl << "Loading model..." << endl;

    // Initialize ROS
    ros::init (argc, argv, "process_pcl");
    ros::NodeHandle nh;

    // Create a ROS publisher for the original and aligned point clouds 
    printf("Create publishers\n");
    transform_pub = nh.advertise<geometry_msgs::Transform> ("transformation", 1);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2> ("aligned_point_cloud", 1);

    // Create a ROS subscriber for the input point cloud
    printf("Crete subscriber\n");
    std::string input_stream; 
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);

     
    // Spin
    ros::Rate r(2); 
    while(ros::ok()) {
      ros::spinOnce();                   // Handle ROS events
      r.sleep();
    }
    return 0;
}
