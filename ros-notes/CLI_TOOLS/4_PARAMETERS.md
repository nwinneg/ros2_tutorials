Notes regarding parameters

# General Idea
- Parameters are used as node configuration values
- Parameters can be: floats, ints, bools, strings, lists
- Each node maintains it's own parameters in ros2
- i.e. member variables in a class instance ... (?)

# Interrogation 
- ros2 param list -- will show a list of parameters for each running node
- Note: 
	- /teleop_turtle and /turtlesim are referred to as "node namespaces"
- Every node contains the "use_sim_time" parameter
- ros2 param get <node_name> <parameter_name> -- will get type and current value
	- Example: ros2 param get /turtlesim background_g

# Action
- ros2 param set <node_name> <parameter_name> <value>
	- Can change a parameter value at runtime
- Doing this will only change the settings for the current session

# Dump params
- We can dump all parameter values out and save them for later
- By default it prints them to the terminal, but we can redirect them to a file
	- ros2 param dump <node_name> - will print to the screen 
	- ros2 param dump <node_name> > <filename.yaml> - will output to a yaml file
	
# Load Params
- ros2 param load <node_name> <parameter_file>
- This allows us to set all the parameters of a node by loading from file

# Loading params on startup
ros2 run <package> <executable> --ros-args --params-file <param_file>
- We can load a parameter file when we run a node by passing the param file as an argument
- e.g. ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim_params.yaml
