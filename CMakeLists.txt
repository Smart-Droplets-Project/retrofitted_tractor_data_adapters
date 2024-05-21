cmake_minimum_required(VERSION 3.5)
project(retrofitted_tractor_data_adapters)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GeographicPose.msg"
  "msg/StateMessage.msg"
  "msg/CommandMessage.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs geographic_msgs std_msgs 
 )

# Python executables
set(PYTHON_EXECUTABLES
    src/ngsild_to_ros2_client.py
)

# Install Python executables
foreach(PYTHON_EXECUTABLE ${PYTHON_EXECUTABLES})
    get_filename_component(EXEC_NAME ${PYTHON_EXECUTABLE} NAME_WE)
    install(PROGRAMS ${PYTHON_EXECUTABLE}
            DESTINATION lib/${PROJECT_NAME}
            RENAME ${EXEC_NAME})
endforeach()

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    src
    DESTINATION lib/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py"
)


#add_executable(cpp_code src/cpp_code.cpp)
## target_compile_definitions(cpp_code PRIVATE <ANY_DEFINITION>)
#target_include_directories(cpp_code PRIVATE include)
#target_compile_options(cpp_code PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
#ament_target_dependencies(cpp_code rclcpp)

#install(TARGETS
#  cpp_code
#  DESTINATION lib/${PROJECT_NAME})

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
