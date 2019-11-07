project( carplanner_msgs )
cmake_minimum_required(VERSION 2.8)
enable_testing()

# Add to module path, so we can find our cmake modules
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )

find_package( catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    tf
    geometry_msgs
    message_generation
)

add_definitions(${EIGEN_DEFINITIONS})

add_message_files(
  DIRECTORY
  msg
  FILES
  Command.msg
  VehicleState.msg
)

generate_messages(
  DEPENDENCIES
  roscpp
  std_msgs
  geometry_msgs

)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
  )

include_directories(
	${CarPlanner_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	)

add_executable( command2cmdvel
  src/command2cmdvel.cpp
)
target_link_libraries( command2cmdvel
  ${catkin_LIBRARIES}
)

add_executable( vehiclestate2pose
  src/vehiclestate2pose.cpp
)
target_link_libraries( vehiclestate2pose
  ${catkin_LIBRARIES}
)
