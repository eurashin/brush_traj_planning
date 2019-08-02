# brush_traj_planning
This package functionality is associated with any point cloud manipulations. This includes actions such as taking a "snapshot" of a depth image through the kinect, surface matching a model to a scene, and trajectory planning on a point cloud. 

## Dependencies: OpenCV 
This package requires OpenCV 3.2.

## Programs 
Note that all instances of "output filename" do not have extensions, because multiple files will be created with the same base string but of different extension types. 
### Take a pointcloud snapshot
```
rosrun brush_traj_planning snapshot [output filename]
```
### Create a model from a colored pointcloud
```
rosrun brush_traj_planning create_model [colored pointcloud.ply] [output filename] [path output directory]
```
### Send trajectories to the ada_server
```
(In another terminal) rosrun ada_server waypoint_server
rosrun brush_traj_planning follow_path [path filename.ply]
```
