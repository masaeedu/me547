cmake_minimum_required(VERSION 2.8.3)
project(board_recognition)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV 2 REQUIRED)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
target_link_libraries(main
  ${OpenCV_LIBRARIES} opencv_nonfree  
)
