cmake_minimum_required(VERSION 3.8)
project(amr_trajectory_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(jsoncpp REQUIRED)

# Generate service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectoryMsg.srv"
  "srv/ReadTrajectoryMsg.srv"
  DEPENDENCIES nav_msgs visualization_msgs
)

# Ensure generated headers are available
include_directories(include)
include_directories(/usr/include/jsoncpp) 
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Add executables
add_executable(trajectory_reader src/trajectory_reader.cpp)
add_executable(trajectory_saver src/trajectory_saver.cpp)

# Link dependencies (remove ${PROJECT_NAME})
ament_target_dependencies(trajectory_reader 
  rclcpp
  nav_msgs
  visualization_msgs
  jsoncpp
)

ament_target_dependencies(trajectory_saver
  rclcpp
  nav_msgs
  visualization_msgs
  jsoncpp
)


target_link_libraries(trajectory_reader jsoncpp)
target_link_libraries(trajectory_saver jsoncpp)

# Link the generated service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_reader ${cpp_typesupport_target})
target_link_libraries(trajectory_saver ${cpp_typesupport_target})

# Install scripts and executables
install(TARGETS
  trajectory_reader
  trajectory_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/trajectory_saver.py
  scripts/trajectory_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
