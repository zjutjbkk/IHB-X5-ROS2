cmake_minimum_required(VERSION 3.8)
project(ihb_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ihb_demo/include/)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/ihb_cpp/ihb_cpp/lib/libsockcanpp/include
)

add_subdirectory(lib/ihb_cpp/ihb_cpp)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ihb_msgs REQUIRED) # CHANGE


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

# add_executable(ihb_debug_node src/main.cpp)
# add_executable(ihb_debug_test src/data_test.cpp)
# 暂时禁用 hall_publisher，因为缺少 SingleArmStatus.msg 定义
add_executable(hall_publisher src/hall_publisher.cpp)
# add_executable(data_upload src/data_upload.cpp)
add_executable(hall_test src/hall_test.cpp)
add_executable(ihb_slave_arm_status_publisher src/ihb_slave_arm_status_publisher.cpp)
# add_executable(hall_test_simple src/hall_test_simple.cpp)
add_executable(ihb_armstatus_publisher src/ihb_armstatus_publisher.cpp)
add_executable(ihb_slave_arm_controller src/ihb_slave_arm_controller.cpp)
add_executable(ihb_master_arm_publisher src/ihb_master_arm_publisher.cpp) 

# target_link_libraries(ihb_debug_node ihb_cpp)
# target_link_libraries(ihb_debug_test ihb_cpp)
target_link_libraries(hall_publisher ihb_cpp)
# target_link_libraries(data_upload ihb_cpp)
target_link_libraries(ihb_slave_arm_status_publisher ihb_cpp)
target_link_libraries(hall_test ihb_cpp)
# target_link_libraries(hall_test_simple ihb_cpp)
target_link_libraries(ihb_armstatus_publisher ihb_cpp)
target_link_libraries(ihb_slave_arm_controller ihb_cpp)
target_link_libraries(ihb_master_arm_publisher ihb_cpp)

# ament_target_dependencies(ihb_debug_node rclcpp  ihb_msgs)
# ament_target_dependencies(ihb_debug_test rclcpp  ihb_msgs)
ament_target_dependencies(hall_publisher rclcpp std_msgs ihb_msgs)
ament_target_dependencies(ihb_slave_arm_status_publisher rclcpp std_msgs ihb_msgs)
# ament_target_dependencies(data_upload rclcpp std_msgs ihb_msgs)
ament_target_dependencies(hall_test rclcpp std_msgs ihb_msgs)
# ament_target_dependencies(hall_test_simple rclcpp std_msgs ihb_msgs)
ament_target_dependencies(ihb_armstatus_publisher  rclcpp std_msgs ihb_msgs)
ament_target_dependencies(ihb_slave_arm_controller  rclcpp std_msgs ihb_msgs)
ament_target_dependencies(ihb_master_arm_publisher  rclcpp std_msgs ihb_msgs)

install(TARGETS
  # ihb_debug_node
  # ihb_debug_test
  hall_publisher
  # data_upload
  hall_test
  # hall_test_simple
  ihb_armstatus_publisher
  ihb_slave_arm_status_publisher
  ihb_slave_arm_controller
  ihb_master_arm_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 安装ihb_cpp库到ROS2包lib目录
install(TARGETS
  ihb_cpp
  DESTINATION lib
)

install(
      DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}
)

ament_package()
