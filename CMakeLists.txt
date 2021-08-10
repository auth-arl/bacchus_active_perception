cmake_minimum_required(VERSION 2.8.3)
project(active_perception)

add_compile_options(-std=c++11)


find_package(PCL 1.3 REQUIRED)

find_package(catkin REQUIRED
  roscpp
  autharl_core
  bacchus_robot
  std_msgs
  cv_bridge
  ur_robot_driver
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  DEPENDS bacchus_robot pcl_conversions
)

include_directories(INCLUDE_DIRS
  include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/active_perception_controller.cpp src/detector.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  bacchus-robot
  armadillo
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/active_perception_controller.cpp src/detector.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  bacchus-robot
  armadillo
)
