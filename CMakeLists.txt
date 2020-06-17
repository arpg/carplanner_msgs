project( carplanner_msgs )
cmake_minimum_required(VERSION 2.8)
enable_testing()

# Add to module path, so we can find our cmake modules
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )

#find_package( carplanner REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    std_srvs
    tf
    geometry_msgs
    nav_msgs
    message_generation
    visualization_msgs
    actionlib_msgs
)

add_definitions(${EIGEN_DEFINITIONS})

add_message_files(
  DIRECTORY
  msg
  FILES
  MotionSample.msg
  Command.msg
  VehicleState.msg
  PathArray.msg
  OdometryArray.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    ResetMesh.srv
    EnableTerrainPlanning.srv
    EnableContinuousPlanning.srv
)

add_action_files(
  DIRECTORY 
    action
  FILES 
    ApplyVelocities.action
#    UpdateState.action 
#    GetGravityCompensation.action 
#    GetFrictionCompensation.action 
    GetControlDelay.action 
    GetInertiaTensor.action
    SetNoDelay.action
    Raycast.action
)

generate_messages(
  DEPENDENCIES
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  carplanner_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime actionlib_msgs nav_msgs
  #LIBRARIES 
  )

include_directories(
	#${CarPlanner_INCLUDE_DIRS}
	#${carplanner_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
	)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

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

add_executable( patharrayparser
  src/patharrayparser.cpp
)
target_link_libraries( patharrayparser
  ${catkin_LIBRARIES}
)

add_executable( waypoint_generator
  src/waypoint_generator.cpp
)
target_link_libraries( waypoint_generator
  ${catkin_LIBRARIES}
)

add_executable( goal_generator
  src/goal_generator.cpp
)
target_link_libraries( goal_generator
  ${catkin_LIBRARIES}
)