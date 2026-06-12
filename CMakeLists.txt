cmake_minimum_required(VERSION 3.8)
project(pointcloud_colorizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)

find_package(PCL REQUIRED)

function(add_colorizer_component target_name)
  add_library(${target_name} SHARED ${ARGN})

  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )

  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)

  ament_target_dependencies(${target_name}
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    nav_msgs
    std_srvs
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

  rclcpp_components_register_nodes(${target_name} ${target_name})
endfunction()

function(add_colorizer_standalone target_name)
  add_executable(${target_name} ${ARGN})

  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )

  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)
  target_compile_definitions(${target_name} PRIVATE POINTCLOUD_COLORIZER_BUILD_STANDALONE)

  ament_target_dependencies(${target_name}
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    nav_msgs
    std_srvs
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

add_colorizer_component(
  RawCloudColorizerColorNode
  src/raw_cloud_colorizer_color_node.cpp
  src/raw_cloud_colorizer_core.cpp
)
add_colorizer_component(
  ColoredCloudMapAggregatorNode
  src/nodes/colored_cloud_map_aggregator_node.cpp
  src/core/colored_cloud_map_aggregator_core.cpp
  src/core/csv_keyframe_pose_provider.cpp
  src/core/voxel_color_estimator.cpp
  src/core/voxel_map_builder.cpp
)

add_colorizer_standalone(
  raw_cloud_colorizer_color
  src/raw_cloud_colorizer_color_node.cpp
  src/raw_cloud_colorizer_core.cpp
)
add_colorizer_standalone(
  raw_cloud_map_aggregator
  src/nodes/colored_cloud_map_aggregator_node.cpp
  src/core/colored_cloud_map_aggregator_core.cpp
  src/core/csv_keyframe_pose_provider.cpp
  src/core/voxel_color_estimator.cpp
  src/core/voxel_map_builder.cpp
)

add_executable(mesh_color_reconstructor src/tools/mesh_color_reconstructor.cpp)
target_include_directories(mesh_color_reconstructor PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(mesh_color_reconstructor PUBLIC cxx_std_17)
target_link_libraries(mesh_color_reconstructor
  ${PCL_LIBRARIES}
)

add_executable(surfel_generator src/tools/surfel_generator.cpp)
target_include_directories(surfel_generator PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(surfel_generator PUBLIC cxx_std_17)
target_link_libraries(surfel_generator
  ${PCL_LIBRARIES}
)

install(TARGETS
  RawCloudColorizerColorNode
  ColoredCloudMapAggregatorNode
  raw_cloud_colorizer_color
  raw_cloud_map_aggregator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  mesh_color_reconstructor
  surfel_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/save_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
