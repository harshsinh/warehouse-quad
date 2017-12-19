cmake_minimum_required(VERSION 2.8.3)
project(pca_linedetector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation
  cmake_modules
)


find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
#find_package(PCL REQUIRED COMPONENTS common io)


add_message_files(
   FILES
   line.msg
 )
 
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   DEPENDS OpenCV Boost Eigen
   INCLUDE_DIRS include/pca_linedetector
   CATKIN_DEPENDS roscpp 
)

include_directories(include/${PROJECT_NAME}
					          ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
                  )
####################
## SubDirectories ##
####################

add_subdirectory (src/pca-linedetector)
