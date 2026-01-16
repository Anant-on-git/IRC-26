cmake_minimum_required(VERSION 3.8)
project(serial_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmPos.msg"
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

ament_package()
