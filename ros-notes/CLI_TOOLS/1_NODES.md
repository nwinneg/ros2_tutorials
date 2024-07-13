Notes about how to interact with nodes

# Remapping
- Remapping is used to pass custom values to reassign properties of a node
- We can reassign things like node names, topic names, service names, etc. 
- for example: 
	- Remapping the node name of the turtlesim node: 
	- ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# Node Information: 
- Nodes store information about things that is accessible on command
	- ros2 node info <node_name>
- This command returns a list of subscribers, publishers, services, and actions

# Summary: 
- Nodes are fundamental components of a ros system
- They are designed to serve a single modular purpose in a robotic system
	- publish sensor data, control a wheel motor, etc.
	
