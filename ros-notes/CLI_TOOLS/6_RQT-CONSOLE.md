Notes on the last few tutorials about CLI tools

## rqt console
- GUI tool used to introspect log messages and organize things over time
- ros2 run rqt_console rqt_console
	- Runs like a node ... 
- Example: 
	- ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
	- This publishes recursively to force the turtle to run into a wall
	- In the console, we see warnings that the turtle hit a wall
[Logger]
- The logger (which we looked at in console) has 5 severity levels
	- Fatal: The system is going to terminate to protect itself
	- Error: Significant issues, won't necessarily damage the system, prevent prop function
	- Warn: Unexpected activity / not-ideal results - don't harm functionality
	- Info: Event / status updates - visual verification of functioning system
	- Debug: Output messages for processes, development tool
- Default level is "info" -- Can filter by severity or other things
	- Note: Console hides msgs below default level unless filtered explicitely
- We can set the default level of a node when we run it
	- Example: ros2 run <package> <executable> --ros-args --log-level WARN
		- This sets the default level to WARN	
- rqt console and the logging capabilites are a useful tool in debug / dev
