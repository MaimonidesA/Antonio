cmake_minimum_required(VERSION 3.8)
project(antonio_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
#find_package(rplidar_ros2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(imu-base_linck-tf2_brodcaster src/imu-base_linck-tf2_brodcaster.cpp)
#ament_target_dependencies(
#    imu-base_linck-tf2_brodcaster
#    geometry_msgs
#    sensor_msgs
#    nav_msgs
#    rclcpp
#    tf2
#    tf2_ros
#)

#add_executable(Wheels_odom_tf2_broadcaster src/Wheels_odom_tf2_broadcaster.cpp)
#ament_target_dependencies(
#    Wheels_odom_tf2_broadcaster
#    geometry_msgs
#    sensor_msgs
#    nav_msgs
#    rclcpp
#    tf2
#    tf2_ros
#)

#add_executable(imu_camera_link-tf2_brodcaster src/imu_camera_link-tf2_brodcaster.cpp)
#ament_target_dependencies(
#  imu_camera_link-tf2_brodcaster
#    geometry_msgs
#    sensor_msgs
#    nav_msgs
#    rclcpp
#    tf2
#    tf2_ros
#)

add_executable(antonio_odom_tf2_broadcaster src/antonio_odom_tf2_broadcaster.cpp)
ament_target_dependencies(
    antonio_odom_tf2_broadcaster
    geometry_msgs
    sensor_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_ros
)

add_executable(antonio_odom 
      src/antonio_odom.cpp
      src/KF.cpp
      )
ament_target_dependencies(
    antonio_odom
    geometry_msgs
    sensor_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_ros
    Eigen3
)

install(
  DIRECTORY src launch rviz config world map
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
    antonio_odom
    antonio_odom_tf2_broadcaster
  #  Wheels_odom_tf2_broadcaster
  #  imu-base_linck-tf2_brodcaster
 #   imu_camera_link-tf2_brodcaster
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
