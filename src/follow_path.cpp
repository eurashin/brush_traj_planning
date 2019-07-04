#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ada_server/WaypointsAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace std; 

void read_path(const char* filename, vector<geometry_msgs::Point> &path) {
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
    pcl::io::loadPLYFile <pcl::PointXYZ> (filename, *cloud);
    
    for(int i=0; i<cloud->points.size(); i++) {
        geometry_msgs::Point point;
        point.x = cloud->points.at(i).x;
        point.y = cloud->points.at(i).y;
        point.z = cloud->points.at(i).z;

        path.push_back(point);
    }
}

bool move_to_waypoint(actionlib::SimpleActionClient<ada_server::WaypointsAction> &ac, geometry_msgs::Point point, geometry_msgs::Quaternion quat) {
    // Send the goal
    ROS_INFO("Action server started, sending goal.");
    ada_server::WaypointsGoal goal;
    geometry_msgs::Pose pose;
    pose.position = point; 
    pose.orientation = quat; 
    goal.end_pose = pose; 
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
      return(true);
    }
    else {
      ROS_INFO("Action did not finish before the time out.");
      return(false);
    }

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
    read_path(argv[1], path);
    cout << "Number of points: " << path.size() << endl;   

    // Initialize waypoint handler
    int loc = 0; 
    cout << "Please hold as still as possible..." << endl;

    geometry_msgs::Point point; 
    geometry_msgs::Quaternion quat;
    
    quat.x = 0; 
    quat.y = 0; 
    quat.z = 0; 
    quat.w = 0; 
    
    for(int i=0; i<path.size(); i++) {
        cout << "moving to point " << i << "..." << endl; 
        point = path[i];  
        move_to_waypoint(ac, point, quat);
    }


    //exit
    return 0;
 
}
