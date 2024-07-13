Notes on how to record data from topics to a database and play back

# Basic Idea
- [ros2 bag] is the cli tool for recording data published to a topic in the system
- Can replay the data or share it with others

# Example
- Run the turtle node and teleop_key
- Create example directory to store data files
	- /home/nwinneg/ros2_humble/turtle_ex/bag_files
- Choose a topic to inspect [/turtle1/cmd_vel]
- ros2 topic echo <topic_name> will print topic stream to the terminal 

# Data Recording
- ros2 bag record <topic_name> will record the data in the the "pwd"
	- This creates a dated folder with two files in it
	- <filename>.db3 - 
	- metadata.yaml - ros version information, topic info, duration/time/msg count 
	
# Record multiple topics
- ros2 bag record -o <bag_file_name> <topic_name_1> <topic_name_2>
	- -o flag allows you to choose a unique name for the bag file
	- Creates a subfolder in the pwd named <bag_file_name> 
		- Contains metadata.yaml and <bag_file_name_#>.db3
- Example: record cmd_vel and pose
	- ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
	
# Interogate (info)
- ros2 bag info <bag_file_name> from a location containing a bag file directory
	- Will return information about the bag files

# Data Playback
- ros2 bag play <bag_file_name> from a location containing a bag file directory
	- Suggested to disable (suspend?) nodes controlling topics to be played back
- Msgs sent to the topics can be @ different rates
	- /cmd_vel only sends msgs when arrows are pressed
	- /pose is updated more frequently --> ros2 topic hz /turtle1/pose to see 
		- 62 hz average
- playback will continue for as long as recorded data
	- Because /pose continuously updates, it will go as long as we collected
	
