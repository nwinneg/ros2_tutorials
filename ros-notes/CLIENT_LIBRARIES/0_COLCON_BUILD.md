Notes regarding using "colcon" tools to build packages

# Basic Idea
- colcon is an iteration on the catkin build tools in ros(1)
	- github.com/colcon
	- sudo apt install python3-colcon-common-extensions (Necessary?)
- A ros workspace: A directory with a particular structure -- typically starts empty
	- [src]: contains source code of ros packages
	- [build]: contains intermediate files -- for ea. pkg, subfolder created in which e.g. CMake is being invoked
	- [install]: location where ea. pkg installed to. By default, ea. pkg to separate directory
	- [log]: contains various logging information about ea. colcon invocation
	
# Creating a workspace
- create a directory to house the workspace: [mkdir -p ~/ros2_ws] and populate it with an src
	- At this point we have only an empty src dir
- For Example, try cloning the ros2 humble examples repository into src
	- git clone https://github.com/ros2/examples src/examples -b humble
	- not src/examples has the ros2 examples
- 

# Overlay / Underlay
- It's important to source the environment for the existing ros2 installation
	- ros2setup -- sources the original install environment
	- This envronment ^ is referred to as the [underlay]
- The new workspace, ros2_ws, will be an [overlay] on top of the existing ros2 installation
	- Generally recommended to use an overlay if iterating on a small number of packages

# Building the workspace
- [ATTN] To build packages in windows, you need to be in a visual studio environment
- Otherwise (Linux)
	- Run 'colcon build' from the root directory of the workspace
	- Optional argument --symlink-install is supported by colcon
- Once we've built the workspace, we can test it using colcon test
- We then need to source the workspace to be able to interact with it
	- source install/setup.bash

