Notes on ros2 topics

# Basic Idea
- Topics act as a bus for data to flow between nodes
- pub/sub
	- Nodes can publish and subscribe to any number of topics 
	
# Visualization
- rqt_graph uses rqt to display the ros graph
- We have turtlesim_node and turtle_teleop_key nodes running
- The graph shows them communicating over topic turtle1/cmd_vel
- It also shows the turtle1/rotate_absolute action

# Cmd line inspection
- ros2 topic list -- displays a list of active topics
- ros2 topic list -t -- displays the same list with types appended

# Cmd line interaction
- echo command lets you see data being published to a topic
	- ros2 topic echo <topic_name>
- Doing this creates a node that subscribes to the topic to listen to it

# Topic info
- ros2 topic info <topic_name> will print a topic's type, publisher count, and subscriber count

# Interface Show -- Msg types
- Nodes communicating over a topic do so by sending messages
- The publishing and subscribed nodes must send/receive the same type of messages
- For example
	- The topic /turtle1/cmd_vel is type geometry_msgs/msg/Twist
- We can get a message type format with: ros2 interface show <msg_type>
- For example: 
	- ros2 interface show geometry_msgs/msg/Twist
	- This tells us to expect a vector of linear components and a vector of angular components in 3D space

# Publishing to a topic
- ros2 topic pub <topic_name> <msg_type> "<args>"
- NOTE: <args> is the actual data to publish -- must be in YAML syntax
- Example: 
	ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" 
- [In practice]
	- A steady stream of msgs is required for continuous operation
	- Replace --once with --rate 1 for continuous streaming
	- Here, rate 1 means it publishes at a 1Hz rate
	ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" 
- This can be seen in the ros graph

# Update rate
- We can inspect the rate at which data is being published
- ros2 topic hz /turtle1/pose
	- This shows ~60Hz (screen refresh rate?)
- If we inspected while publishing to turtle1/cmd_vel at 1Hz, the average would be about that


