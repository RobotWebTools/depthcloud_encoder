cmake_minimum_required(VERSION 2.8.3)
project(depthcloud_encoder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cv_bridge sensor_msgs image_transport message_filters roscpp)
find_package(Boost REQUIRED COMPONENTS system thread)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS cv_bridge sensor_msgs image_transport message_filters roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)
                    
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

## Declare a cpp executables and libraries
add_library(${PROJECT_NAME}_lib
   src/depthcloud_encoder.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
   src/depthcloud_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
