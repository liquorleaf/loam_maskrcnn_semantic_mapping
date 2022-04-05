cmake_minimum_required(VERSION 3.1.3)
project(loam_velodyne_semantic_mapping)

ADD_DEFINITIONS(-DOPENCV)
ADD_DEFINITIONS(-DGPU)

find_package(catkin REQUIRED COMPONENTS
cv_bridge
geometry_msgs
nav_msgs
sensor_msgs
roscpp
rospy
std_msgs
tf
pcl_conversions)

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs sensor_msgs roscpp rospy std_msgs tf pcl_conversions
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES loam
)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${Eigen3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_definitions( -march=native ) # 修改：加上这一行会报错，所以注释掉

add_library(loam
            src/lib/math_utils.h
            src/lib/BasicSemanticMapping.cpp
            src/lib/SemanticMapping.cpp
            src/lib/ScanRegistration.cpp
            src/lib/BasicScanRegistration.cpp
            src/lib/MultiScanRegistration.cpp
            src/lib/LaserOdometry.cpp
            src/lib/BasicLaserOdometry.cpp
            src/lib/LaserMapping.cpp
            src/lib/BasicLaserMapping.cpp
            src/lib/TransformMaintenance.cpp
            src/lib/BasicTransformMaintenance.cpp)
target_link_libraries(loam ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(semanticMapping src/semantic_mapping_node.cpp)
target_link_libraries(semanticMapping loam ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(multiScanRegistration src/multi_scan_registration_node.cpp)
target_link_libraries(multiScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserOdometry src/laser_odometry_node.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserMapping src/laser_mapping_node.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(transformMaintenance src/transform_maintenance_node.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

#if (CATKIN_ENABLE_TESTING)  # 下载测试数据并用rostest做多节点测试
#  find_package(rostest REQUIRED)
#  # TODO: Download test data
#  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
#    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
#    MD5 3d5194e6981975588b7a93caebf79ba4)
#   add_custom_target(${PROJECT_NAME}_test_data
#     COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
#     DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
#   configure_file(tests/loam.test.in
#                  ${PROJECT_BINARY_DIR}/test/loam.test)
#   add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
#     DEPENDENCIES
#       ${PROJECT_NAME}_test_data
#       multiScanRegistration
#       laserOdometry
#       laserMapping
#       transformMaintenance)
# endif()


SET(CMAKE_BUILD_TYPE Debug)
