Notes on how to launch multiple nodes at once from CLI

# Basic Idea
- As we scale systems, opening each node one at a time is annoying
- Launch files let you start up and configure many executables containing ros2 nodes
	- ros2 launch
	- Starts the entire system at once -- all nodes and their configurations

# Example: 
- ros2 launch turtlesim multisim.launch.py
- This lives in: install/turtlesim/share/turtlesim/launch/multisim.launch.py
- Note: 
	- This one is in python, XML and YAML can also be used to create launch files
	- This might be helpful if programmatically creating them with config changes
	
- Launch files will be touched on later -- intermediate / Launch in the tutorials

ros2 topic pub -r 1 turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

from launch import LaunchDescription
import launch_ros.actions


# Python launch code

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
      ])

