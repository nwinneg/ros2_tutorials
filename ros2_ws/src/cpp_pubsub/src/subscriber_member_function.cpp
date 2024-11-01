#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using std::placeholders::_1; 

class MinimalSubscriber : public rclcpp::Node
{
public:
	// Set the class constructor for subclass of Node
	MinimalSubscriber()
	: Node("minimal_subscriber")
	{
		// Just set the one member variable
		// Subscribe to topic, bind a callback, 
		subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(
		  "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
	}

private:
	// Create the callback function on message reception
	void topic_callback(const tutorial_interfaces::msg::Num & msg) const
	{
		// Just log the received message to the console
		RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
	}
	// Initialize the the member variable for a subscription
	rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};
	
// Create a main to run the node 
int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv); // Init ros2 with args
	rclcpp::spin(std::make_shared<MinimalSubscriber>()); // Spin the node 
	rclcpp::shutdown();
	return 0;
}
