Notes on initial introductiont to actions in ros2

# Basic Idea
- Communication type intended for long running tasks
- Built on both topics and services
- Uses an client/server model where: 
	- The action client node sends a goal to the action server node
	- The action server node: 
		- Acknowledges the goal via response to goal service
		- Sends steady stream of feedback via feedback topic
		- Returns a result via response to result service
In total we have: 
	- Action client node: goal service client, feedback subscriber, result service client
	- Action server node: goal service server, feedback publisher, result service server
	- Action: Goal service, feedback topic, result

# Using Actions
- In turtle_teleop_key, the absolute orientation key commands are implemented as actions
	- Clicking a key sends a request to a goal server to rotate to that position
	- Once the goal is completed, it sends a response to the request service to say so
- Note: 
	- When a new goal is received before finishing current, it aborts current
	- This is not always the case -- should not be assumed
	
# Inspection
- ros2 node info <node-name> -- will display info about topic subscribes / publishes and who servers and clients are for services and actions
	- rotate_absolute is listed as action server for turtlesim 
	- it is listed as an action client for teleop_turtle
- ros2 action list -- returns a list of actions in current session
	- -t will return the action type
- ros2 action info <action_name>
	- Returns more information about the structure of the action
- ros2 interface show <action_type>
	- Returns the structure of the action type -- needed for calls
	
# Sending goals from CLI
- ros2 action send_goal <action-name> <action-type> <values>
	- Values must be in yaml format
- When doing this, we get the following in response: 
	- It waits for the action server to become available
	- Notice that the goal was sent and what it was
	- Goal acknowledgement with unqiue ID (all goals have a unique ID)
	- The result and final status
- To see the feedback as the goal server is working, add --feedback to end of line

# Summary
- Actions operate like services that allow for long running tasks, provide feedback, and are cancelable
- A good example of actions is for navigation
	- Goal sent to robot to navigate to a position
	- Robot sends regular updates along the way and returns result when arrives

