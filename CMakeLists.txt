cmake_minimum_required(VERSION 3.8)
project(epuck_driver_cpp_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# building
add_executable(${PROJECT_NAME}_node src/epuck_driver_cpp_ros2.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)


# Installing node
install(TARGETS
${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME}
)

# Installing launch files
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

# Installing config files
install(FILES
	config/epuck_driver_cpp_ros2.yaml
  	DESTINATION share/${PROJECT_NAME}/config/
)

ament_target_dependencies(${PROJECT_NAME}_node 
rclcpp
sensor_msgs
nav_msgs
geometry_msgs
std_msgs
tf2_ros
)

ament_package()