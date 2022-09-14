cmake_minimum_required(VERSION 3.8)
project(led_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/TurnLED.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

 
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_library(serialib src/serialib.cpp)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp) 
rosidl_target_interfaces(server 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(led_client src/led_client.cpp)
ament_target_dependencies(led_client rclcpp)
rosidl_target_interfaces(led_client 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")




target_link_libraries(server serialib)


install(
TARGETS server led_client     
DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS serialib
  EXPORT seriarialib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
