cmake_minimum_required(VERSION 3.8)
project(my_robot_pos)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(GTSAM REQUIRED)

# add gtsam
add_executable(my_robot_pos src/robot_pos.cpp)
target_link_libraries(my_robot_pos gtsam)
ament_target_dependencies(my_robot_pos rclcpp std_msgs geometry_msgs nav_msgs)

install(TARGETS
my_robot_pos
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
