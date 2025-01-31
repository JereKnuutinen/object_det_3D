cmake_minimum_required(VERSION 3.5)
project(gnss_pose_estimation)

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
find_package(tf2_ros REQUIRED)
include_directories(${tf2_ros_INCLUDE_DIRS})
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(gnss_pose_estimation_node src/gnss_pose_estimation_node.cpp)


ament_target_dependencies(gnss_pose_estimation_node
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "visualization_msgs"
  "PCL"
)

include_directories(/home/mlemsalu/object_det_3D/ll2enu)
target_include_directories(gnss_pose_estimation_node PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(gnss_pose_estimation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


##target_include_directories(NMEA2000_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

install(TARGETS gnss_pose_estimation_node
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
