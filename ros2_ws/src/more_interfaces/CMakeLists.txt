cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ========= Interface Dependencies ==========
# Generates msg code from msg/srv files
find_package(rosidl_default_generators REQUIRED)  
# Declare the list of messages we want to generate
set(msg_files
   "msg/AddressBook.msg"
)
# Ensure CMake knows to reconfigure the project when we add new msg files
rosidl_generate_interfaces(${PROJECT_NAME}
 ${msg_files}
)
# Ensure to export package runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# ========= Source Code Dependencies ==========
find_package(rclcpp)
add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)
install(TARGETS
	publish_address_book
	DESTINATION lib/${PROJECT_NAME})

# ========= Required for including interface and source code in common pkg ===========
rosidl_get_typesupport_target(cpp_typesupport_target	
	${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(publish_address_book "${cpp_typesupport_target}")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
