Notes regarding more complex and interesting custom interfaces we could integrate

# Basic Idea: 
- We've looked at the most basic msg and srv type interfaces
- Best practices are to create custom interface in their own dedicated package
	- However, it can be useful to declare, create, and use an interface all in one package
- It is possible to have python libraries and nodes in a CMake package
	- This allows us to create python nodes and interfaces all in one package
- We'll use C++ nodes here for simplicity and will focus on more complex msg type interfaces

# Create a new package
- Create a package called more_interfaces to house our new interfaces [from src]
	- ros2 pkg create --build-type ament_cmake --licence Apache-2.0 more_interfaces
	- mkdir more_interfaces/msg
- Create a new message to house our "Address book"
	- AddressBook.msg
		- Add options for phone types [integer indexed]
		- Store data for first/last name, phone number, and phone type
	- Add rosidl default dependencies to package.xml file
		- <buildtool_depend>rosidl_default_generators</buildtool_depend>
  		- <exec_depend>rosidl_default_runtime</exec_depend>
  		- <member_of_group>rosidl_interface_packages</member_of_group>
	- Add dependences to CMakeLists.txt file as well
		- # Generates msg code from msg/srv files
		  	find_package(rosidl_default_generators REQUIRED)  	
		- # Declare the list of messages we want to generate
			set(msg_files
			   "msg/AddressBook.msg"
			)
		- # Ensure CMake knows to reconfigure the project when we add new msg files
			rosidl_generate_interfaces(${PROJECT_NAME}
			 ${msg_files}
			)
		- # Ensure to export package runtime dependencies
			amend_export_dependencies(rosidl_default_runtime)
- Write a cpp file to start using our AddressBook message
	- Create a src file called publish_address_book.cpp
	- Create a node and publish the address book periodically
	- See code for comments

- Make updates to CMakeLists.txt
	- Create a new target for this node in CMakeLists
		find_package(rclcpp)
		add_executable(publish_address_book src/publish_address_book.cpp)
		ament_target_dependencies(publish_address_book rclcpp)
		install(TARGETS
			publish_address_book
			DESTINATION lib/${PROJECT_NAME})
	- In order for the messages to generate in the same package, need the following too
	- This finds relevent C++ code for AddressBook.msg and allows target to link against it
	- Only necessary when using interfaces in the same package where they are defined
		rosidl_get_typesupport_target(cpp_typesupport_target	
		  ${PROJECT_NAME} rosidl_typesupport_cpp)
		target_link_libraries(publish_address_book "${cpp_typesupport_target}")

# Test it out
- Build it
- Run it
	- [from one terminal] ros2 run more_interfaces publish_address_book
	- [from another term] ros2 topic echo /address_book
- Another thing we could do for fun is to write a simple subscriber that subscribes to this topic

		
