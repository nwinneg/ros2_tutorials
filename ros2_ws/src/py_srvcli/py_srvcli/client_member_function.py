import sys  # TBD why we need this

from example_interfaces.srv import AddTwoInts # Need the request fmt
import rclpy
from rclpy.node import Node


# Create a subclass of Node -- Asyncronous apparently
class MinimalClientAsync(Node):
	
	# Create class constructor
	def __init__(self):
		super().__init__('minimal_client_async')
		self.cli = self.create_client(		# Member function of Node
					AddTwoInts,	# Request type (must match service)
					'add_two_ints') # Request name (must match service)
		# Check to see if the service is available, timeout after 1 second
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		# If available, make the request
		self.req = AddTwoInts.Request()
	
	# Create a member function to send a request
	def send_request(self, a, b):
		self.req.a = a
		self.req.b = b
		# create a "future" that can be passed to "spin until future complete"
		# Seems like "future" tells "spin until future complete" when response received
		return self.cli.call_async(self.req) 
		
# Create the main function -- this is a little more complex than before
def main():
	rclpy.init()
	
	minimal_client = MinimalClientAsync() # Create instance of the client
	future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2])) # Make request
	rclpy.spin_until_future_complete(minimal_client, future)
	response = future.result() # Report the result when received
	minimal_client.get_logger().info(
		'Result of add_two_ints: %d + %d = %d' %
		(int(sys.argv[1]), int(sys.argv[2]), response.sum))
	
	minimal_client.destroy_node()
	rclpy.shutdown()
	

if __name__ == '__main__':
	main()
	
	
