Notes regarding creating a ROS2 package

# Basic Idea
- Goal: Create a ros2 package using either CMake or python and run it's executable
	- [ATTN] We will follow the python instructions but cmake would be useful maybe
- Package creation uses [ament] as the build system and [colcon] as the build tool

# Make-up of a package
- The package directory goes in /src
- For a python built package, we need the following minimum files: 
	- package.xml			: file containing meta info about the package
	- /resource/<package_name>	: marker file for the package
	- setup.cfg			: required if have executables so ros2 run can find
	- setup.py			: instructions for how to install the package
	- <package_name>		: contains __init__.py, used by ros2 to find package
- Packages in a workspace
	- A workspace can contain arbitrarily many packages -- each gets it's own directory
	- Can mix and match build type packages (cmake, py) but not nested

# Creating a new package
- We'll create it in the ros2_ws workspace we already have (in src)
- Package will be called "my_package"
- The command to create a new package (in python) is: 
	- ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
- We will include --node-name my_node (before the package name)
	- This creates a simple hello world executable called my_node
- This command autogenerates the minimal set of files set of files
- We can now build the whole workspace -- building all of the contained packages
	- colcon build
- To selectively build packages within a workspace: 
	- colcon build --packages-select my_package
	- Useful if I only make changes to my package
- The executable my_node lives in: 
	- src/my_package/my_package/my_node.py

# Customizing package.xml
- Some fields are not automatically set by the pkg build tools:
	- description, license, maintainer
	- "Necessary" if you want to release your package
- If doing anything but using myself, probably use the MIT open source license ... 
- <*_depend> tags are where we would list dependencies on other packages
	- We don't have any yet but may need them later ... 
- [setup.py] contains the same fields (description, license, maintainer) which we should edit
	- [ATTN] In this file, package name and version must match package.xml exactly










