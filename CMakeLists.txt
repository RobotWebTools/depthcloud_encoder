cmake_minimum_required(VERSION 2.8)
project(pointcloud_image_streamer)

find_package(catkin REQUIRED)
catkin_package()

find_package(catkin REQUIRED sensor_msgs image_transport message_filters tf)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} pointcloud_image_streamer/src/pointcloud_image.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} 
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

