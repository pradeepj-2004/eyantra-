cmake_minimum_required(VERSION 3.8)
project(ur5_moveit)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}

  "srv/Tfvalue.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)


#rosidl_generate_interfaces(${PROJECT_NAME}
#
 # DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
#)

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



#ament_python_install_package(pymoveit2)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  DIRECTORY
    srv
    scripts
  DESTINATION
    share/${PROJECT_NAME}/
)

set(EXAMPLES_DIR examples)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/task1b.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/task2a_4b.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/tf_box.py
		scripts/task4b_endeff.py
		scripts/tool0_service.py
 DESTINATION lib/${PROJECT_NAME})

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
ament_package()
