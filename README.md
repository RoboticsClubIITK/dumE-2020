# DumE-test
## Building Files
Add all the folders to your catkin workspace and then run <code>catkin build</code> to build the packages.
## Running Simulation
Run <code>roslaunch kinect_robo default.launch</code> to start the simulation.
## Additional Information
* Visualisation of camera data can be seen in RViz
* color image data is published to topic <code>/kinect/color/image_raw</code>
* depth image data is published to topic <code>/kinect/depth/image_raw</code>
* pointcloud data is published to topic <code>/kinect/depths/points</code>
