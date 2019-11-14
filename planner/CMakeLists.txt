cmake_minimum_required(VERSION 3.5)
project(planner)

set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        nav_msgs
        message_generation
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
        roscpp
        geometry_msgs
        nav_msgs
        nav_msgs
        message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_LIBRARIES SHARED
        src/Planner.cpp
        src/Graph.cpp
)

add_executable(rrt_planner src/rrt_planner_node.cpp src/RRTPlanner.cpp)
target_link_libraries(rrt_planner ${PROJECT_NAME}_LIBRARIES ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(rrt_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
