Notes regarding defining custom msg and srv formats for node communication

# Basic Idea
- It is sometimes useful to define a message or srv request/response format
- This example walks through creating a basic one
- These formats live in their own package -- e.g. example_interfaces is a package

# Create a new package for them
- We'll call it tutorial_interfaces
	- Create it with no executables or dependencies for now
- from src, create the pkg
	- Interface packages must be done using CMake but can be used by C++ or Python nodes
	- ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_interfaces
- This gives us the following files: 
	- LICENSE		: license description
	- CMakeLists.txt	: Describes how to build the code within the package
	- package.xml		: Meta info about the package - same as python ish
	- src/			: directory containing package source code
- Create subfolders for msg and srv [required]
	- from /ros2_ws/src/tutorial_interfaces, mkdir msg srv

# Creating custom message definition
- number type
	- Create a file called Num.msg containing
		- int64 num
- Sphere type
	- Create a file called Sphere.msg containing
		- geometry_msgs/Point center
		- float64 radius
	- The sphere center uses a msg type from another package (geometry_msgs) as it's type

# Creating custom service definition
- Add three ints service
	- file called AddThreeInts.srv containing: 
		- int64 a
		- int64 b
		- int64 c
		---
		- int64 sum
	- a, b, and c are the arguments and sum is the response

# Update CMakeLists.txt
- Convert interfaces into language specific code (C++ or Python)
- Add lines to the dependencies section to ensure we have packages we need
	- find_package(geometry_msgs REQUIRED)
	- find_package(rosidl_default_generators REQUIRED)
- Add add additional lines to generate the new interfaces: 
	- rosidl_generate_interfaces(${PROJECT_NAME}
		"msg/Num.msg"
		"msg/Sphere.msg"
		"srv/AddThreeInts.srv"
		DEPENDENCIES geometry_msgs # Add any packages that the new interfaces depend on
	  )
	- Some confusion about ${PROJECT_NAME} -- tbd if it works
	- Newer ros2 distros seem to suggest creating interfaces in a file ... (?) tbd

# Update package.xml file
- <depend>geometry_msgs</depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
- This does the following: 
	- Adds the dependency package
	- Defines an extra build tool dependency
	- Sets an executable dependency
	- This package is a dependency group of rosidl_interface_packages so it defines that

# Build the package and confirm creation
- colcon build --packages-select tutorial_interfaces
	- Seems to have been successful
- ros2 interface show tutorial_interfaces/msg/Sphere

# Testing the interfaces with the Publisher/Subscriber 
- We could modify the py_pubsub and py_srvcli packages we created previously
- Here, we will create C++ packages instead and use our template_interfaces instead of example
	- Creating a C++ package syntax
		- ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub
	- Create a cpp file for the talker in the cpp_pubsub/src directory 
		- Write the talker node (publisher_member_function.cpp)
		- See notes in code
		- Mainly the same -- more complex in C++, but just change msg used in callback
	- Create a cpp file for the listener in the cpp_pubsub/src directory
		- Write the listener node (subscriber_member_function.cpp)
		- See notes in code
		- Mainly the same again -- more c++ specifics to familiarize with
	- Add dependencies to CMakeLists.txt
		- find_package(rclcpp REQUIRED)
		- same for tutorial_interfaces
	
		- add_executable(talker src/publisher_member_function.cpp)
		- ament_target_dependencies(talker rclcpp tutorial_interfaces)
		* Repeat for listener
		
		- Add the following before ament_package() 
			- This helps ros2 run find the executables
		- install(TARGETS
			talker
			listener
			DESTINATION lib/&{PROJECT_NAME})
	- Add dependency to package.xml
		- <depend>tutorial_interfaces</depend>

# Build and run
- Many C++ errors -- fixed -- now the package will build properly
- source the underlay if new window [ros2setup]
- source the overlay if new window [source install/setup.bash]
- ros2 run cpp_pubsub talker
- ros2 run cpp_pubsub listener

# Testing the interfaces with the Server/Client
- Here we will do the same as we did with the Publisher/Subscriber and create a new package to house a CPP version of a server/client -- modified to use the custom interfaces.
	- Creating a C++ package syntax
		- ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_srvcli --dependencies rclcpp tutorial_interfaces
	- Check package.xml
		- Don't have to manually add dependencies because of the way we called the package creation
		- Should check the description and maintainer lines
		- Validate msg package dependency
	- Create a cpp file for the service node
		- in cpp_srvcli/src create add_three_ints_server.cpp
		- See comments in code for notes
	- Create a cpp file for the client 
		- in cpp_srvcli/src create add_three_ints_client.cpp
		- See comments in code for notes
	- Add dependencies to CMakeLists.txt
		- add executables, target dependencies, and install list
			ament_target_dependencies(server 
				rclcpp tutorial_interfaces)

			add_executable(client src/add_three_ints_client.cpp)
			ament_target_dependencies(client 
				rclcpp tutorial_interfaces)
				
			install(TARGETS
			  server
			  client
			  DESTINATION lib/${PROJECT_NAME})
# Build and run
- colcon build --packages-select cpp_srvcli
- Oof errors
- Success :) just semicolons and commas and stuff
- ros2 run cpp_srvcli server
	- Ready and waiting
- ros2 run cpp_srvcli client 3 2 1
	- Not available, waiting
	- Sum: 6
	
# In summary
- We've now integrated a custom interface package into pubsub and srvcli packages
- Next, we'll think about more ways to use interfaces?










