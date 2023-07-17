cmake_minimum_required(VERSION 3.8)
project(robot_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(robot_interface REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/Fusion.msg")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include Serial ${EIGEN3_INCLUDE_DIR})
add_library(Estimator src/Estimator.cpp)
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp src/RobotInterface.cpp Serial/CRC/CRC.cpp)
target_link_libraries(${PROJECT_NAME}_node Estimator)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp nav_msgs geometry_msgs sensor_msgs std_srvs tf2_ros tf2_eigen robot_interface)

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})


ament_package()
