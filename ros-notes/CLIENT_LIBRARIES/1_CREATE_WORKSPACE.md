Notes regarding creating a ROS2 workspace

# Basic Idea
- When working with ros2, we always have to source the main installation workspace
- We have the option of sourcing an additional workspace or [overlay] on top of the existing one or [underlay]
- The overlay can be used to add new packages without interfering with the existing ros2 workspace

# Creating a new workspace
- Mainly starts by just creating some directory and adding a [src] directory to it
	- mkdir -p ~/ros2_ws/src
- Best practice is to put packages into the src directory (source code)

# Add packages to src
- For now, we'll add the turtlesim package to the development workspace
- git clone https://github.com/ros/ros_tutorials.git -b humble
	- Grabbing the branch for the humble ros2 distro
	- Cloned into src

# Resolve Dependencies
- Best practice is to check dependencies when cloning or adding a package
- From the root directory ros2_ws
	- rosdep install -i --from-path src --rosdistro humble -y
	- Should see [#All required rosdeps installed successfully]
- The above command does the following
	- Walks through called out dependencies in a package's package.xml file
	- It installs the missing ones

# Build the workspace
- colcon build --symlink-install
- colcon test
- Source the underlay (main ros workspace) 
	- source /opt/ros/humble/setup.bash (ros2setup)
- Source the overlay (from ros2_ws)
	- source /install/local_setup.bash
- We can now run the turtlesim package from the overlay

# Modifying the overlay
- We can modify the overlay seperately from the underlay and the overlay takes precedence
- For example, lets modify the title bar in the turtlesim node window
	- This is in turtleframe.cpp -- edit it in the source code (src)
	- Modify the function setWindowTitle()
	- Rebuild the workspace
- Now: 
	- If the overlay is sourced, those changes will take effect
	- If only the underlay is sourced, they will not be present








