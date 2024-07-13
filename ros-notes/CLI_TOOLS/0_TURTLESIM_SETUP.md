Notes on turtlesim exampls

# Setup and run turtle
- source the workspace and download turtlesum
	- ros2setup, sudo apt update/upgrade, sudo apt install ros-humble-turtlesim
- Check the package was installed
	- ros2 pkg executables turtlesim
	- ^ lists available executables in turtlesim
- Start turtlesim
	[ros2 run <package> <executable>]
	- ros2 run turtlesim turtlesim_node
	- Sim window appears showing the turtle ... 
- Open a new terminal to do more stuff
	- ros2 run turtlesim turtle_teleop_key	
	- Can operate the turtle with arrow keys and rotation command keys ... 
- To look at more things: 
	- ros2 node list: Shows nodes we have running, right now only /turtlesim
	- ros2 topic list: Shows available topics, three come from turtle
	- ros2 service list: TBA
	- ros2 action list: TBA

# RTQ
- Can graphically interact and call services 
	- Spawn a new turtle using the /spawn service

# Remapping topic
- We have a topic for turtle1/cmdvel that is used by turtle_teleop_key to control turtle1
- We want, instead, for turtle_teleop_key to control turtle2/cmdvel
- We can remap which topic is called by passing an argument in the call
- ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

# End of tutorial 1


