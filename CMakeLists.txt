cmake_minimum_required(VERSION 3.10)
project(anti_drone_system)

set(CMAKE_CXX_STANDARD 17)

# Gazebo
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# ===========================
# Camera Bridge
# ===========================

add_executable(gz_cam_bridge src/gz_cam.cpp)

ament_target_dependencies(gz_cam_bridge
    rclcpp
    sensor_msgs
    cv_bridge
)

target_include_directories(gz_cam_bridge PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(gz_cam_bridge
    gz-transport12::core
    gz-msgs9::core
    ${OpenCV_LIBS}
)

# ===========================
# IMU Bridge
# ===========================

add_executable(imu_pub src/imu_pub.cpp)

ament_target_dependencies(imu_pub
    rclcpp
    sensor_msgs
)

target_link_libraries(imu_pub
    gz-transport12::core
    gz-msgs9::core
)

# ===========================
# ZED Bridge
# ===========================

add_executable(zed_ros_cam
    src/zed_ros_cam.cpp
)

ament_target_dependencies(zed_ros_cam
    rclcpp
    sensor_msgs
    cv_bridge
)

target_include_directories(zed_ros_cam PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(zed_ros_cam
    gz-transport12::core
    gz-msgs9::core
    ${OpenCV_LIBS}
)

#================================
# Livox Mid360
#================================

add_executable(livox_ros src/livox_ros.cpp)

ament_target_dependencies(livox_ros
    rclcpp
    sensor_msgs
)

target_link_libraries(livox_ros
    gz-transport12::core
    gz-msgs9::core
)



# ----------------------

install(TARGETS
    gz_cam_bridge
    imu_pub
    zed_ros_cam
    livox_ros
    DESTINATION lib/${PROJECT_NAME}
)
