cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometer src/odometer.cpp)
target_link_libraries(odometer ${catkin_LIBRARIES})

add_executable(gps_odometer src/gps_odometer.cpp)
target_link_libraries(gps_odometer ${catkin_LIBRARIES})
add_dependencies(gps_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

