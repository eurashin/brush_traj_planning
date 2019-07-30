#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ada_server/WaypointsAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "cloud_tools.h"
using namespace std; 

void read_path(const char* filename, vector<geometry_msgs::Point> &path, vector<geometry_msgs::Point> &normals) {
    pcl::PointCloud <pcl::PointNormal>::Ptr raw_cloud (new pcl::PointCloud <pcl::PointNormal>);
    pcl::io::loadPLYFile <pcl::PointNormal> (filename, *raw_cloud);
    
    pcl::PointCloud <pcl::PointNormal>::Ptr cloud (new pcl::PointCloud <pcl::PointNormal>);
    transform_pointcloud(*raw_cloud, *cloud);
    
    for(int i=0; i<cloud->points.size(); i++) {
        geometry_msgs::Point point;
        point.x = cloud->points.at(i).x;
        point.y = cloud->points.at(i).y;
        point.z = cloud->points.at(i).z;
        
        geometry_msgs::Point normal;
        normal.x = cloud->points.at(i).normal_x;
        normal.y = cloud->points.at(i).normal_y;
        normal.z = cloud->points.at(i).normal_z;

        path.push_back(point);
        normals.push_back(normal);
    }
}

void transform_pose(geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &new_pose) {
    // Get the tf from /depth_optical_frame to /map
    tf::TransformListener tf_listener;

    try {
        tf_listener.waitForTransform("/map", "/camera_depth_optical_frame", ros::Time::now(), ros::Duration(1.0));
        tf_listener.transformPose("/map", ros::Time::now(), pose, pose.header.frame_id, new_pose);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

bool send_waypoints(actionlib::SimpleActionClient<ada_server::WaypointsAction> &ac, vector<geometry_msgs::Point> path, vector<geometry_msgs::Point> normals) {
    geometry_msgs::Quaternion quat;
    quat.x = 0; // Identity quaternion 
    quat.y = 0.7071; 
    quat.z = 0; 
    quat.w = 0.7071; 
    tf::Vector3 up_vector(0, 0, 1); // Pointing straight up  
    
    vector<geometry_msgs::PoseStamped> new_path; 
    int increment = path.size()/4; 
    for(int i=path.size() - 1; i>0; i-=increment) {
        ROS_INFO("Moving to point %d...", i);
        geometry_msgs::Point p = path.at(i); 
        geometry_msgs::Point np = path.at(i - 1); 
        geometry_msgs::Point norm = normals.at(i); 
        // Find second vector
        tf::Vector3 axis_vector(p.x - np.x, p.y - np.y, p.z - np.z);
        //tf::Vector3 axis_vector(norm.x, norm.y, norm.z);
        
        // Get rotation between two vectors
        /*
        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
        q.normalize();
        tf::quaternionTFToMsg(q, quat);
        */
        // Send the goal
        ROS_INFO("Action server started, sending goal.");

        geometry_msgs::Pose pose;
        pose.position = p;
        pose.position.x -= (0.039); // Account for the distance of the comb
        pose.orientation = quat;

        std_msgs::Header header; 
        header.stamp = ros::Time::now(); 
        header.frame_id = "camera_depth_optical_frame";
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header; 
        pose_stamped.pose = pose; 
        
        ROS_INFO("(%f, %f, %f | %f, %f, %f, %f)", p.x, p.y, p.z, quat.x, quat.y, quat.z, quat.w);     
        new_path.push_back(pose_stamped);
    }
        ada_server::WaypointsGoal goal;
        goal.path = new_path; 

        geometry_msgs::Transform trans; 
        goal.transformation = trans;
        ac.sendGoal(goal);
}


int main (int argc, char** argv) {
    // Start a client
    ros::init(argc, argv, "test_trajectory");
    actionlib::SimpleActionClient<ada_server::WaypointsAction> ac("waypoints", true);

    // Wait for server
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    // Read PLY point path to follow
    vector<geometry_msgs::Point> path;
    vector<geometry_msgs::Point> normals; 
    read_path(argv[1], path, normals);
    cout << "Number of points: " << path.size() << endl;   

    // Initialize waypoint handler
    int loc = 0; 
    cout << "Please hold as still as possible..." << endl;
    send_waypoints(ac, path, normals);

    //exit
    return 0;
 
}
