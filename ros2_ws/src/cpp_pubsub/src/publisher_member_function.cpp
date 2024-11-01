#include <chrono>
#include <memory>
//#include <string>
//#include <functional>

#include "rclcpp/rclcpp.hpp"	   	// Library containing common pieces of ros2 system
#include "tutorial_interfaces/msg/num.hpp" 	// Our library containing custom msg type
// #include "std_msgs/msg/string.hpp" 	

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node // Create a class that inherits from Node
{					     // NOTE: each 'this' refers to the Node
public:
	// Create the class constructor for the publisher
	MinimalPublisher()
	: Node("minimal_publisher"), count_(0) // Initializing count_ to zero
	{
		// Initialize publisher with msg type and topic name
		publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic",10);
		// Initialize a timer that triggers our publishing callback
		timer_ = this->create_wall_timer(
		// Here, std::bind registers the member function as a callback
		500ms, std::bind(&MinimalPublisher::timer_callback, this));
	}
	
private:
	// Create the timer callback
	void timer_callback()
	{
		// Set message to the num type
		auto message = tutorial_interfaces::msg::Num();
		message.num = this->count_++; // count up
		// Use rclcpp to print to console
		RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");
		// Publish msg to topic
		publisher_->publish(message);
	}
	// Initialize timer, publisher, and count variables with their types
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
	size_t count_;
};


// Create a main function to actually run the node
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv); // initializes ros2
	rclcpp::spin(std::make_shared<MinimalPublisher>()); // start processing node data
	rclcpp::shutdown(); // kill the node when done
	return 0;
}














		
