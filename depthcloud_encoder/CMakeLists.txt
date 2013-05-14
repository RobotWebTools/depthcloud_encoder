cmake_minimum_required(VERSION 2.8)
project(depthcloud_encoder)

find_package(catkin REQUIRED cv_bridge sensor_msgs image_transport message_filters roscpp)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS cv_bridge sensor_msgs image_transport message_filters roscpp
)

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS}
                    )
                    
link_directories( ${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_lib
   src/depthcloud_encoder.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(${PROJECT_NAME}_nodelet
   node/depthcloud_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_node
   node/depthcloud_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

 install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES depthcloud_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
