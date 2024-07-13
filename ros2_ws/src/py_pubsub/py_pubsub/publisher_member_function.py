import rclpy
from rclpy.node import Node # We need the node class

from std_msgs.msg import String # built-in ros2 string message type

# Note: this is a subclass of (inherits from) the class Node
class MinimalPublisher(Node):

	# Create the class constructor
	def __init__(self):
		# Calls Node class constructor with my node name
		super().__init__('minimal_publisher')
		# create_publisher declares that the node publishes String msgs 
		# Publishes over a topic names 'topic' with queue size 10
		self.publisher_ = self.create_publisher(String, 'topic', 10)
		# Set timer period to be 0.5s
		timer_period = 0.5
		# Creates a timer to execute timer_callback every timer_period seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		# Initialize a counter to be used in the callback
		self.i = 0
		
	# Create the timer callback
	def timer_callback(self):
		# Define msg to be a string type (String must be a class)
		msg = String()
		# Populate msg data with a msg and append the counter we initialized
		msg.data = 'Hello World: %d' % self.i
		# Publish the data using our publisher (to 'topic')
		self.publisher_.publish(msg)
		# Use get_logger to print to the console the data we've published
		self.get_logger().info('Publishing: "%s"' % msg.data)
		# Increment the member variable counter
		self.i += 1
		
# Initialize a main function to run from if __name__ == '__main__':
def main(args=None):
	# initialize the rclpy library
	rclpy.init(args=args)
	# create an instance of our publisher node
	minimal_publisher = MinimalPublisher()
	# "Spin" the node so the callbacks are called 
	rclpy.spin(minimal_publisher)
	
	
	# Destroy the node explicitely
	# (optional - happens automatically when garbage collection destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()

# Actually run the main
if __name__ == '__main__':
	main()
