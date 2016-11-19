cmake_minimum_required(VERSION 2.8.3)
project(board_recognition)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV 2 REQUIRED)

catkin_package()

include_directories(
  include/board_recognition
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB main_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(main ${main_SRC})

target_link_libraries(main
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} opencv_nonfree  
)
