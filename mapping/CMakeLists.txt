cmake_minimum_required(VERSION 3.5)
project(mapping)

set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        message_generation
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(mapping_node src/mapping_node.cpp src/Map.cpp)
target_link_libraries(mapping_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
