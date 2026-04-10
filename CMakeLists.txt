cmake_minimum_required(VERSION 3.8)
project(pointcloud_colorizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(PCL REQUIRED)

function(add_colorizer_executable target_name source_file)
  add_executable(${target_name} ${source_file})

  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )

  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)

  ament_target_dependencies(${target_name}
    rclcpp
    sensor_msgs
    nav_msgs
    pcl_conversions
    cv_bridge
    tf2_ros
    tf2
    geometry_msgs
    message_filters
  )

  target_link_libraries(${target_name}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
endfunction()

add_colorizer_executable(raw_cloud_colorizer src/raw_cloud_colorizer_node.cpp)
add_colorizer_executable(registered_cloud_colorizer src/registered_cloud_colorizer_node.cpp)

install(TARGETS
  raw_cloud_colorizer
  registered_cloud_colorizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
