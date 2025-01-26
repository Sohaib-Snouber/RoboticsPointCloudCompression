cmake_minimum_required(VERSION 3.8)
project(point_cloud_compression)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(original_point_cloud src/original_point_cloud.cpp)
ament_target_dependencies(original_point_cloud rclcpp sensor_msgs pcl_conversions pcl_msgs)
target_link_libraries(original_point_cloud ${PCL_LIBRARIES})

add_executable(voxel_grid_filtering src/voxel_grid_filtering.cpp)
ament_target_dependencies(voxel_grid_filtering rclcpp sensor_msgs pcl_conversions pcl_msgs)
target_link_libraries(voxel_grid_filtering ${PCL_LIBRARIES})

add_executable(voxel_grid_decompression src/voxel_grid_decompression.cpp)
ament_target_dependencies(voxel_grid_decompression rclcpp sensor_msgs pcl_conversions pcl_msgs)
target_link_libraries(voxel_grid_decompression ${PCL_LIBRARIES})

add_executable(octree_compression src/octree_compression.cpp)
ament_target_dependencies(octree_compression rclcpp sensor_msgs pcl_conversions pcl_msgs)
target_link_libraries(octree_compression ${PCL_LIBRARIES})

add_executable(octree_decompression src/octree_decompression.cpp)
ament_target_dependencies(octree_decompression rclcpp sensor_msgs pcl_conversions std_msgs)
target_link_libraries(octree_decompression ${PCL_LIBRARIES})

add_executable(save_point_cloud src/save_point_cloud.cpp)
ament_target_dependencies(save_point_cloud rclcpp sensor_msgs pcl_conversions)
install(TARGETS save_point_cloud DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  original_point_cloud
  voxel_grid_filtering
  octree_compression
  octree_decompression
  voxel_grid_decompression
  save_point_cloud
  DESTINATION lib/${PROJECT_NAME})


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

