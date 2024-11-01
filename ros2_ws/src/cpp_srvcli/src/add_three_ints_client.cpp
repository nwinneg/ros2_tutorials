#include "rclcpp/rclcpp.hpp" // standard ros2 C++ client library
#include "tutorial_interfaces/srv/add_three_ints.hpp" // Our new msg type library

#include "chrono"
#include "cstdlib"
#include "memory"

using namespace std::chrono_literals;

// Main only
int main(int argc, char **argv)
{
	// Instantiate 
	rclcpp::init(argc,argv);
	
	// Check that we have the appropriate number of arguments
	if (argc != 4) { // argc represents the number of command line arguments passed
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");
	}
	
	// Create a node to house our client
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");

	// Create a client for that node of the appropriate msg type
	rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client = 
	  node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");
	
	// Create the request in the appropriate format given by the .srv
	auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();
	
	// Add the command line arguments to the request structure
	request->a = atoll(argv[1]);
	request->b = atoll(argv[2]);
	request->c = atoll(argv[3]);
	
	// Wait for 1s for the service node to become available
	while (!client->wait_for_service(1s)) { // wait for one second before continuing
	  if (!rclcpp::ok()) { // if interupted somehow, say so by throwing an error
	    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service, exiting.");
	    return 0;
	  }
	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
	}
	
	// Assuming the service is available by now, sent the request to the service
	auto result = client->async_send_request(request);
	
	// Wait for the result
	if (rclcpp::spin_until_future_complete(node,result) == 
	  rclcpp::FutureReturnCode::SUCCESS) { // Once return code indicates a successful response, do things
	    // Print the result
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld",result.get()->sum);
	} else { // If return code is not success
	    // Indicate failure to call service
	    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");
	}
	
	// Shutdown the node
	rclcpp::shutdown();
	return 0;
	 
}

