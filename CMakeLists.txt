cmake_minimum_required(VERSION 3.8)
project(pointcloud_colorizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)

add_executable(my_node src/my_node.cpp)

target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)

target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(my_node 
  rclcpp 
  sensor_msgs 
  pcl_conversions
)

target_link_libraries(my_node ${PCL_LIBRARIES})

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
