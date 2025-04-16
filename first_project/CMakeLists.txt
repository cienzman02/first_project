cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  tf
  message_generation
)

add_message_files(
  FILES
  sector_times.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


#catkin_package()
catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometer src/odometer.cpp)
target_link_libraries(odometer ${catkin_LIBRARIES})

add_executable(gps_odometer src/gps_odometer.cpp)
target_link_libraries(gps_odometer ${catkin_LIBRARIES})
add_dependencies(gps_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sector_times src/sector_times.cpp)
target_link_libraries(sector_times ${catkin_LIBRARIES})

add_dependencies(sector_times first_project_generate_messages_cpp)
add_dependencies(sector_times ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})