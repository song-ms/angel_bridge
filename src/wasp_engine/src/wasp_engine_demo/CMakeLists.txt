cmake_minimum_required(VERSION 3.5)
project(wasp_engine_demo)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(include)

add_library(eth_packet SHARED src/eth_packet.cc)

### cm emulator
add_executable(cm_emulator src/cm_emulator.cc src/eth_packet.cc)

### fake eth bridge
add_executable(fake_eth_bridge src/fake_eth_bridge.cc)

### eth bridge
add_executable(eth_bridge src/eth_bridge.cc src/eth_packet.cc)
# target_link_libraries(eth_bridge eth_packet)

### test codes
add_executable(test_eth_packet src/test_eth_packet.cc)
target_link_libraries(test_eth_packet eth_packet)
# add_executable(test_tcpip_client src/test_tcpip_client.cc)
# target_link_libraries(test_tcpip_client eth_packet)

# ament_export_include_directories(include)
# ament_export_libraries(eth_packet)

ament_target_dependencies(fake_eth_bridge
  rclcpp
  std_msgs
  trajectory_msgs
)

ament_target_dependencies(eth_bridge
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  std_srvs
)

install(TARGETS
  cm_emulator
  fake_eth_bridge
  eth_bridge
  # eth_packet
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
