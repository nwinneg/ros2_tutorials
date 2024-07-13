Notes on ros2 services

# Basic Idea: 
- Services are another way of communicating between nodes, based on call-and-response
	- As opposed to publish/subscribe
	- Services only provide data when called by a client
- There an be many clients using the same service but only one server
	- Critically, only one node can provide the response e.g. serve the request
- We can check the list of services with the following: 
	- ros2 service list <opt>
- Nearly all ros2 nodes have the same set of 6 services with "parameters" in the name
	- describe, get_parameter_types, get_parameters, list_parameters, set_parameters, set_parameters_automatically.
	
# Service Type
- interrogate with: ros2 service type <service_name> 
- For example: 
	- the /clear service is type std_srvs/srv/Empty
	- The empty type means it sends no data when requesting and receives no data when receiving a response
- Searching: ros2 service find <type_name>
	- We can search for services of a specific type
	- e.g. ros2 service find std_srvs/srv/Empty
- Interface Show: ros2 interface show <type_name>
	- This tells us the arguments required in a service call (request)
	- For example: turtlesim/srv/Spawn
		- Interface show says we need an x, y, theta, and optionally name
		
		
# Service Calls: 
- We can make service calls from the cmd line
	- ros2 service call <service_name> <service_type> <arguments>
- For example: Calling empty service
	- ros2 service call /clear std_srvs/srv/Empty
	- This calls the /clear service to delete the pen lines behind the turtle(s)
- For example: Calling /spawn service with parameters
	- ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}" 
	- The service immediately creates the new turtle and returns the response
	- We now have a turtle1 topic and a turtle2 topic
- Note: 
	- Here, turtle2/cmd_vel has been created as a topic
	- It is a 'leaf topic' in the rqt graph
	- There is nothing that points to it, it only points to turtlesim
	- THis is because turtle_teleop_key only publishes to turtle1/cmd_vel
	
# Summary: 
- Only one node can act as the service provider (server)
- Client nodes can request to the server node which process the request and returns a response
- Topics (or actions) are preferable if continuous calls are required

