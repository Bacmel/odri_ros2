cmake_minimum_required(VERSION 3.8)
project(odri_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(odri_msgs REQUIRED)
find_package(odri_control_interface REQUIRED)

set(ODRI_INTERFACE_SRC src/robot_interface.cpp)
set(ODRI_STATE_MACHINE_SRC src/robot_state_machine.cpp)

include_directories(include ${ODRI_CONTROL_INTERFACE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(robot_interface ${ODRI_INTERFACE_SRC})
add_executable(robot_state_machine ${ODRI_STATE_MACHINE_SRC})
ament_target_dependencies(robot_interface rclcpp odri_msgs)
ament_target_dependencies(robot_state_machine rclcpp odri_msgs)
target_link_libraries(robot_interface odri_control_interface::odri_control_interface)
target_link_libraries(robot_state_machine odri_control_interface::odri_control_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS robot_interface DESTINATION lib/${PROJECT_NAME})
install(TARGETS robot_state_machine DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
