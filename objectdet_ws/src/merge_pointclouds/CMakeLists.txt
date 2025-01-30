cmake_minimum_required(VERSION 3.5)
project(merge_pointclouds)

# Default to C99
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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(${tf2_ros_INCLUDE_DIRS})
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(merge_pointclouds_node src/merge_pointclouds_node.cpp)


ament_target_dependencies(merge_pointclouds_node
  "rclcpp"
  "std_msgs"
  "custom_msgs"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "geometry_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "PCL"
)

#target_link_libraries(merge_pointclouds_node tf2_eigen)
target_include_directories(merge_pointclouds_node PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(merge_pointclouds_node ${PCL_LIBRARIES})
target_include_directories(merge_pointclouds_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


##target_include_directories(NMEA2000_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

install(TARGETS merge_pointclouds_node
  DESTINATION lib/${PROJECT_NAME})

 # install(TARGETS NMEA2000_node
 # DESTINATION lib/${PROJECT_NAME})

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
