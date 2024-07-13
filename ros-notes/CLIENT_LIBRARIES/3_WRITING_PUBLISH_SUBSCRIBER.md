Notes regarding writing a basic publish subscriber package

[NOTE] This will be done in python, there is a module for doing it with C++ which might be useful to do later if I do this on a machine with a C++ IDE

# Basic Idea
- Creating nodes that pass information over a topic (talker and listener system)
- One node publishes data and the other subscribes to the topic and receives the data

# Create the package
- First we'll create a new package in the ros2_ws workspace called py_pubsub
	- In this case, we don't need --node-name my_node
- We should get the standard output from package creation
- [NOTE] here we can edit the description files as needed -- skipping for now

# Writing the publisher node
- Nav to ros2_ws/src/py_pubsub/py_pubsub
- Recommended downloading a standard talker node -- I'm going to try writing (copying) one
	- publisher_member_function.py
	- See comments in code for description

# Adding Dependencies
- We need to do some editing of the package description files
	- Description: Example of a minimal publisher/subscriber using rclpy
- For this publisher executable we wrote, we need to add the following lines: 
	- <exec_depend>rclpy</exec_depend>
	- <exec_depend>std_msgs</exec_depend>

# Add entry point to setup.py script
- Open the setup.py script and add the following under entry_points={
	- 'talker = py_pubsub.publisher_member_function:main',
- Assuming this lets us run the executable from the console ... (?)

# Check setup.cfg
- It should look like this: 
	[develop]
	script_dir=$base/lib/py_pubsub
	[install]
	install_scripts=$base/lib/py_pubsub
- This just tells setuptools to put the executables in /lib/ because ros2 run will look there

[ATTN] We could build, source, and run now but we should build the subscriber node first ... 

# Writing the subscriber node
- Nav to ros2_ws/src/py_pubsub/py_pubsub again
- I'm gonna write it myself again because I'm an idiot
	- subscriber_member_function.py
	- See comments in code for description
- Dependencies:
	- This script has the same dependencies so we don't need to add more
	- We do need to check dependencies: 
		- rosdep install -i --from-path src --rosdistro humble -y
- Add the entry point
	- We need to do the same thing -- this time we'll call it listener
- Now we can try building, sourcing, and running
	- [ATTN] If there are issues in package executable code, building will throw errors
		- It does however tell you what they are and you can go fix them
		- I had some open/close brackets and missing : to go and find :)

# Running the nodes
- Since we set the console entry points to be "talker" and "listener" that's how we run them
	- ros2 run py_pubsub talker
	
	
	
	
	
	
	
	
