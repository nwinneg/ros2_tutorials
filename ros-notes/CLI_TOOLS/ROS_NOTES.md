## Jumble of notes as I learn more things: 

# General notes about running things and doing examples --> Mostly from links at the bottom
- ign topic : check the topics provided by gazebo 
- ros2 topic list : check the topics available / currently running by ros
- ros_gz_bridge : is a package that allows exchange of messages between ROS and Gazebo
	- `source /opt/ros/humble/setup.bash` is required before doing this stuff it seems
	- Launching one of these bridges is done by: 
		- ros2 run ros_gz_bridge parameter_bridge ...
			- I don't really know what `parameter_bridge` does yet
	- We then have to tell it what model we're interested in talking to and what to do
		- /models/ refers to a model in the sdf
		- /vehicle_blue/ is a particular model
		- /cmd_vel since we're giving velocity commands
	-We then have an @ symbol and tell it what type of messages we'll be sending
		- We first give it the ROS2 message type: geometry_msgs/msg/Twist
		- We then separate with a `]`
		- And give it the gazebo message type: ignition.msg.Twist
	- This initializes the bridge and allows communication via the topic I set up
	- Alternatively: 
		- I can set up a topic and bridge explicitely with a YAML configuration file
		
- This is a little different when interacting with sensors: 
	- The '[' goes the other direction for one --> Seems to define data flow direction 
		- ROS]IGN is ROS --> IGN
		- ROS[IGN is ROS <-- IGN
	- There's also some extra stuff like remapping topics here
		- e.g. remapping /lidar2 topic to /lidar_scan topic with /lidar2:=laser_scan
		- Still not totally sure how this all works
		
- I think we're at a stage where we can start to go through some video tutorials and do a little more tooth cutting

# Once started up do the following to get going
- ros2setup - sources the ros2 workspace
- ignsetup - sources appropriate ign and gazebo stuff
- Test that it worked
	- terminal one: ros2 run demo_nodes_cpp talker [Also works with demo_nodes_py]
	- terminal two: ros2 run demo_nodes_cpp listener 
- 

# Notes: 
- 'ros2 pkg prefix <package>' will return the path of that package

- Links: 
	- docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
	- github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge


# Example SDF Files: 
- File location: /usr/share/ignition/ignition-gazebo6/worlds
- This is where I can find example SDF files for the majority of examples that exist
