cmake_minimum_required(VERSION 3.8)
project(final_group1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# create a variable for all dependencies
set(RWA5_INCLUDE_DEPENDS
  rclcpp 
  std_msgs
  ariac_msgs
  tf2_ros
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  geometric_shapes
  moveit_ros_planning_interface
  robot_msgs
)

# find all dependencies
foreach(dependency IN ITEMS ${RWA5_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

#-----------------------------
# C++
#-----------------------------


add_executable(moveit_demo_cpp_python_exe src/floor_robot_cpp_python.cpp src/main_cpp_python.cpp)
ament_target_dependencies(moveit_demo_cpp_python_exe ${RWA5_INCLUDE_DEPENDS})

install(TARGETS moveit_demo_cpp_python_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY
  include
  launch
  config
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})
# ament_python_install_package(${PROJECT_NAME}
#     SCRIPTS_DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS
moveit_demo_cpp_python_exe
DESTINATION lib/${PROJECT_NAME}
)

# install Python executables
install(PROGRAMS 
    script/competition_node.py
    script/kitting_node.py
    DESTINATION lib/${PROJECT_NAME}
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
