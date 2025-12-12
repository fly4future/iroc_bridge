cmake_minimum_required(VERSION 3.28)
project(iroc_bridge)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[iroc_bridge]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  rclcpp_action
  mrs_msgs
  mrs_lib
  mrs_robot_diagnostics
  sensor_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  IROCBridge 
)

## --------------------------------------------------------------

## |                           Compile                          |
## --------------------------------------------------------------

# Estimation Manager

add_library(IROCBridge SHARED
  src/iroc_bridge.cpp
)

target_include_directories(IROCBridge
  PRIVATE
    "include"
)

target_link_libraries(IROCBridge
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_action::rclcpp_action
    mrs_lib::mrs_lib
    mrs_robot_diagnostics::mrs_robot_diagnostics
    ${mrs_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
)

rclcpp_components_register_nodes(IROCBridge "iroc_bridge::IROCBridge")

target_compile_definitions(IROCBridge PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
