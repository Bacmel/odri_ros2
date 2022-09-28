cmake_minimum_required(VERSION 3.8)
project(odri_ros2_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(odri_ros2_msgs REQUIRED)
find_package(odri_control_interface REQUIRED)

# ---- MOTOR TEST LIBRARY ----
list(APPEND ODRI_INTERFACE_TOOLS_SRC src/robot_interface.cpp)
list(APPEND ODRI_INTERFACE_TOOLS_SRC src/robot_state_machine.cpp)
list(APPEND ODRI_INTERFACE_TOOLS_SRC src/odri_ros2_interface.cpp)

add_library(odri_interface_tools SHARED ${ODRI_INTERFACE_TOOLS_SRC})

target_include_directories(
  odri_interface_tools PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(odri_interface_tools rclcpp odri_ros2_msgs)
target_link_libraries(odri_interface_tools
  odri_control_interface::odri_control_interface)

list(APPEND ${PROJECT_NAME}_INSTALL_TARGETS odri_interface_tools)

# ---- ROBOT INTERFACE NODE ----
list(APPEND ROBOT_INTERFACE_SRC src/nodes/robot_interface.cpp)

set(ROBOT_INTERFACE_EXEC_NAME robot_interface)

add_executable(${ROBOT_INTERFACE_EXEC_NAME} ${ROBOT_INTERFACE_SRC})
target_link_libraries(${ROBOT_INTERFACE_EXEC_NAME} odri_interface_tools)

list(APPEND ${PROJECT_NAME}_INSTALL_TARGETS ${ROBOT_INTERFACE_EXEC_NAME})

# ---- ROBOT STATE MACHINE NODE ----
list(APPEND ROBOT_STATE_MACHINE_SRC src/nodes/robot_state_machine.cpp)

set(ROBOT_STATE_MACHINE_EXEC_NAME robot_state_machine)

add_executable(${ROBOT_STATE_MACHINE_EXEC_NAME} ${ROBOT_STATE_MACHINE_SRC})
target_link_libraries(${ROBOT_STATE_MACHINE_EXEC_NAME} odri_interface_tools)

list(APPEND ${PROJECT_NAME}_INSTALL_TARGETS ${ROBOT_STATE_MACHINE_EXEC_NAME})

# ---- ODRI ROS2 INTERFACE NODE ----
list(APPEND ODRI_ROS2_INTERFACE_SRC src/nodes/odri_ros2_interface.cpp)

set(ODRI_ROS2_INTERFACE_EXEC_NAME odri_ros2_interface)

add_executable(${ODRI_ROS2_INTERFACE_EXEC_NAME} ${ODRI_ROS2_INTERFACE_SRC})
target_link_libraries(${ODRI_ROS2_INTERFACE_EXEC_NAME} odri_interface_tools)

list(APPEND ${PROJECT_NAME}_INSTALL_TARGETS ${ODRI_ROS2_INTERFACE_EXEC_NAME})

# ---- INSTALL TARGETS ----
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_export_dependencies(odri_control_interface)
ament_export_dependencies(odri_ros2_msgs)

install(DIRECTORY include DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${${PROJECT_NAME}_INSTALL_TARGETS}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include
  DESTINATION lib)

ament_package()
