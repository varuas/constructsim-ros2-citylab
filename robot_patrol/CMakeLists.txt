cmake_minimum_required(VERSION 3.5)
project(robot_patrol)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  DEPENDENCIES 
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)

add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service rclcpp std_msgs sensor_msgs std_srvs custom_interfaces)

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service rclcpp std_msgs sensor_msgs std_srvs custom_interfaces)

add_executable(patrol_with_service src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service rclcpp std_msgs sensor_msgs geometry_msgs std_srvs custom_interfaces)

add_executable(go_to_pose_action src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs std_srvs tf2 rclcpp_action)
rosidl_target_interfaces(go_to_pose_action
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(go_to_pose_action "${cpp_typesupport_target}")

install(TARGETS
   patrol_node
   direction_service
   test_service
   patrol_with_service
   go_to_pose_action
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
