cmake_minimum_required(VERSION 3.0.2)
project(place_recog_eval)
set(CMAKE_CXX_STANDARD 14)
set(PYTHON_VERSION 3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
message("CATKIN_PACKAGE_BIN_DESTINATION: ${CATKIN_PACKAGE_BIN_DESTINATION}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  cv_bridge
  image_transport
  tf_conversions
  livox_ros_driver
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions pcl_ros sensor_msgs geometry_msgs nav_msgs message_filters cv_bridge image_transport
  DEPENDS Boost PCL OpenCV
)

include_directories(
  include
  include/methods/
  include/methods/scan_context/
  include/methods/STD/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add source files to a variable
set(EVALUATE_SRC
  src/evaluate.cpp
  src/evaluator.cpp
  src/methods/dbow.cpp
  src/methods/scan_context.cpp
  src/methods/scan_context/Scancontext.cpp
)


add_executable(evaluate ${EVALUATE_SRC})
target_link_libraries(evaluate
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${EIGEN3_LIBRARIES}
DBoW2
)

add_executable(merge_messages src/merge_messages.cpp)
target_link_libraries(merge_messages
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${EIGEN3_LIBRARIES}
)

add_executable(msg_converter src/msg_converter.cpp)
target_link_libraries(msg_converter
${catkin_LIBRARIES}
${PCL_LIBRARIES}
)

add_executable(dbow_create_vocab src/dbow_create_vocab.cpp src/methods/dbow.cpp)
target_link_libraries(dbow_create_vocab
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
  DBoW2
)

add_executable(place_recognition_livox src/methods/place_recognition_livox.cpp src/methods/STD/STDesc.cpp )
target_link_libraries(place_recognition_livox 
        ${catkin_LIBRARIES} 
        ${PCL_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${CERES_LIBRARIES})