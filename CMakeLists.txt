cmake_minimum_required(VERSION 3.8)
project(system_data_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

add_library(sdr_component SHARED src/sdr_component.cpp)
target_compile_definitions(sdr_component PRIVATE "SDR_BUILDING_DLL")

# Detect ROS 2 distribution for API compatibility.
# In Humble, rosbag2_transport/qos.hpp exists and TopicMetadata.offered_qos_profiles is
# a YAML string. In Jazzy+, the header was removed (Rosbag2QoS moved to rosbag2_storage)
# and TopicMetadata.offered_qos_profiles became std::vector<rclcpp::QoS>.
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  target_compile_definitions(sdr_component PRIVATE "ROS_DISTRO_HUMBLE")
endif()
ament_target_dependencies(sdr_component
  lifecycle_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  yaml_cpp_vendor
)
rclcpp_components_register_nodes(sdr_component "sdr::SystemDataRecorder")

add_executable(system_data_recorder src/system_data_recorder.cpp)
target_link_libraries(system_data_recorder sdr_component)
ament_target_dependencies(system_data_recorder
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
)

install(TARGETS
  sdr_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  system_data_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
