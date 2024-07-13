Notes regarding writing a basic service client package

[NOTE] This will be done in python, there is a module for doing it with C++ which might be useful to do later if I do this on a machine with a C++ IDE

# Basic Idea
- Goal: Create a package that has service and client nodes in python
- Reminders about service/client communication:
	- The client node sends a request for data to the service node
	- The service node responds to the request
	- The structure of the request and response is determined by a .srv file
- This example is basic addition: one node requests the sum of two ints and the other responds

# Create the package: 
- We will call this one py_srvcli
- This time we'll automatically add the dependencies: 
	- This is done with the args: --dependencies rclpy example_interfaces after pkg_name
	- example_interfaces is the package that contains the .srv file to structure requests
- Full cmd: ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces
- Update package.xml and setup.py descriptions (And other stuff :/)
	- Python client server tutorial

# Write the service node
- Nav to src/py_srvcli/py_srvcli
	- service_member_function.py
	- See code comments for notes
- Add the cli entry point to setup.py 
	- 'service = py_srvcli.service_member_function:main',

# Write the client node
- Nav to src/py_srvcli/py_srvcli
	- client_member_function.py
	- See code comments for notes
- Add the cli entry point to setup.py
	- 'client = py_srvcli.client_member_function:main',
- [ATTN] Script names should NOT have a file extension
	
# Build and run
- Check dependencies
	- rosdep install -i --from-path src --rosdistro humble -y
- Build
	- colcon build --package-select py_srvcli
- Open a new term, source underlay and overlay, and run the service
	- This should just sit until called
	- When called, prints the incoming request
- In another terminal, source again and run: 
	- ros2 run py_srvcli client a b where a and b are numbers
	- These are sys arguments being passed as system arguments (sys.argv[])

# Next to create custom interfaces :)
