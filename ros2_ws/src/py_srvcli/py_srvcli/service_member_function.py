from example_interfaces.srv import AddTwoInts # Import the srv formatting file

import rclpy
from rclpy.node import Node  # Will need to make a subclass of Node


class MinimalService(Node):

	# Create the class constructor for subclass of Node
	def __init__(self):
		super().__init__('minimal_service')
		self.srv = self.create_service(			# Member function of Node
				AddTwoInts,			# Needs a service type
				'add_two_ints',			# Name the service
				self.add_two_ints_callback)	# Callback to do when requested
	
	# Create the service request callback
	def add_two_ints_callback(self,request,response):
		response.sum = request.a + request.b  # Perform the operation
		self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
		
		return response
	
# Create a main function
def main():
	rclpy.init() # initialize rclpy
	
	minimal_service = MinimalService() # Create instance of our service
	
	rclpy.spin(minimal_service) # Spin the node to get it going
	
	rclpy.shutdown()
	
	
if __name__ == '__main__':
	main()
		
		
