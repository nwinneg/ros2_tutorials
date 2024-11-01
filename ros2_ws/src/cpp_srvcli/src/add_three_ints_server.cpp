#include "rclcpp/rclcpp.hpp" 				// standard
#include "tutorial_interfaces/srv/add_three_ints.hpp"	// our custom service header file

#include "memory" // not sure

// Function to do the addition
void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request, // request type
	 std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response> response)	 // response type
{
	// -> operator used to access members of struct or class
	response->sum = request->a + request->b + request->c; 
	
	// Use the standard logger to print the incoming request
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming Request\na: %ld" "b: %ld" "c: %ld",
			request->a, request->b, request->c);
	
	// User the standard logger to print the response to send
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending response: [%ld]", (long int)response->sum);
	
}

// Main
int main(int argc, char **argv)
{
	// Instantiate ros2 cpp client library
	rclcpp::init(argc,argv);
	
	// Create a node named add_three_ints_server
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");
	
	// Create service for node named add_three_ints and advertise it over network with the &add function
	rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service = 
	 node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",&add);
	
	// Print a log message when the service is ready
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");
	
	// Spin the node to make the service available and shutdown
	rclcpp::spin(node);
	rclcpp::shutdown();
}
