cmake_minimum_required(VERSION 3.8)
project(bacok)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

add_library(dubinsPath SHARED src/lib/dubinsPath.cpp)
ament_target_dependencies(dubinsPath)
target_include_directories(dubinsPath PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(frameTransport SHARED src/lib/frameTransport.cpp)
ament_target_dependencies(frameTransport Eigen3)
target_include_directories(frameTransport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


add_executable(attitude_control src/attitude_control.cpp)
ament_target_dependencies(attitude_control rclcpp px4_msgs)
install(TARGETS attitude_control
  DESTINATION lib/${PROJECT_NAME})

add_executable(takeoff src/takeoff.cpp)
ament_target_dependencies(takeoff rclcpp px4_msgs)
install(TARGETS takeoff
  DESTINATION lib/${PROJECT_NAME})

add_executable(processTelem src/processTelem.cpp)
ament_target_dependencies(processTelem rclcpp px4_msgs)
install(TARGETS processTelem
  DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_dubins src/offboard_dubins.cpp)
ament_target_dependencies(offboard_dubins rclcpp px4_msgs)
target_link_libraries(offboard_dubins dubinsPath frameTransport)
install(TARGETS offboard_dubins
  DESTINATION lib/${PROJECT_NAME})


ament_export_include_directories(include)
ament_export_libraries(dubinsPath)
ament_export_libraries(frameTransport)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
  
install(TARGETS dubinsPath
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(TARGETS frameTransport
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)


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

ament_package()
