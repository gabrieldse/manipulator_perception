cmake_minimum_required(VERSION 3.8)
project(ring_tracker_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveCamera.srv"
  "srv/GrabRing.srv"
  DEPENDENCIES  # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
