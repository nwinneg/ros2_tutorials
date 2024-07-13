import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# Create a subclass of Node for our subscriber
class MinimalSubscriber(Node):
	
	# Create the constructor for our class
	def __init__(self):
		super().__init__('minimal_subscriber')
		self.subscription = self.create_subscription( # Member function of Node
			String,   # Message type must match what the publisher is publishing
			'topic',  # We want to subscribe to the correct topic to communicate
			self.listener_callback, # Callback executes when msg is received
			10)			# Buffer for speed deficiency handling 
		self.subscription # Prevents unused variable warning
		
	# Create the listener callback
	def listener_callback(self, msg):
		# Just print the message that was received
		self.get_logger().info('I heard: "%s"' % msg.data)
		
# Create the main function 
def main(args=None):
	# Initialize rclpy
	rclpy.init(args=args)
	# Create instance of our subscriber class
	minimal_subscriber = MinimalSubscriber()
	# 'Spin' the node to get the callback going
	rclpy.spin(minimal_subscriber)
	
	# Destroy when done
	minimal_subscriber.destroy_node()
	rclpy.shutdown()



if __name__ == '__main__':
	main()
		
