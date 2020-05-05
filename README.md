# DumE-test
## Setting up Kinect model
* Add the <code>kinect</code> directory to <code>\~/.gazebo/models</code> (create <code>models</code> sub-directory if it doesn't exist).  
* Also type the following command into your terminal window  
<code>echo 'export GAZEBO_MODEL_PATH=\~/.gazebo/models:$GAZEBO_MODEL_PATH' >> \~/.bashrc</code> (for bash users)  
<code>echo 'export GAZEBO_MODEL_PATH=\~/.gazebo/models:$GAZEBO_MODEL_PATH' >> \~/.zshrc</code> (for zsh users)  
## Building Files
Add all the folders except <code>kinect</code> to your catkin workspace and then run <code>catkin build</code> to build the packages.
## Running Simulation
Run <code>roslaunch kinect_robo default.launch</code> to start the simulation.
## Additional Information
* Visualisation of camera data can be seen in RViz
* color image data is published to topic <code>/kinect/color/image_raw</code>
* depth image data is published to topic <code>/kinect/depth/image_raw</code>
* pointcloud data is published to topic <code>/kinect/depths/points</code>
