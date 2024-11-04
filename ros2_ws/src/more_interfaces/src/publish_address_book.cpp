#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <more_interfaces/msg/address_book.hpp> // new msg header

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node // Create a class that inherits from rclcpp's Node class
{
public: 
  AddressBookPublisher() // Create an instance of that class to house our node
  : Node("address_book_publisher")
  {
    address_book_publisher_ = // Create the publisher object for the node
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book",10);
    
    auto publish_msg = [this]() -> void { // Create a callback function to publish periodically
   
      auto message = more_interfaces::msg::AddressBook(); // Create msg instance to publish later
      
      // populate message fields
      message.first_name = "John";
      message.last_name = "Doe";
      message.phone_number = "1234567890";
      message.phone_type = message.PHONE_TYPE_MOBILE;
      
      // Send out the message
      std::cout << "Publishing Contact\nFirst: " << message.first_name <<  // Write to terminal
        "  Last: " << message.last_name << std::endl;
        
      this->address_book_publisher_->publish(message); // Publish the message to a topic
      
    }; // End of publish_msg callback 
    
    // Create a timer to publish the message every 1 second
    timer_ = this->create_wall_timer(1s, publish_msg);
      
  }
  
private: 
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Do Main
int main(int argc, char * argv[])
{
  // Initialize rclcpp
  rclcpp::init(argc,argv);
  
  // Spin the node to get things going
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  
  // Shut it down when it's done
  rclcpp::shutdown();
  
  return 0;
  
}
  
    
