cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(
  Eigen REQUIRED
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES lee_position_controller position_controller mpc_controller crazyflie_onboard_controller
    roll_pitch_yawrate_thrust_controller sensfusion6 crazyflie_complementary_filter eigen_helpers
    mellinger_controller internal_model_controller
    roll_pitch_yawrate_thrust_controller_crazyflie
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(lee_position_controller
  src/library/lee_position_controller.cpp
)

add_library(position_controller
   src/library/position_controller.cpp
)

add_library(mpc_controller
   src/library/mpc_controller.cpp
)

add_library(crazyflie_onboard_controller
   src/library/crazyflie_onboard_controller.cpp
)

add_library(roll_pitch_yawrate_thrust_controller
  src/library/roll_pitch_yawrate_thrust_controller.cpp
)

add_library(roll_pitch_yawrate_thrust_controller_crazyflie
  src/library/roll_pitch_yawrate_thrust_crazyflie.cpp
)

add_library(crazyflie_complementary_filter
  src/library/crazyflie_complementary_filter.cpp
)

add_library(eigen_helpers
  src/library/eigen_helpers.cpp
)

add_library(sensfusion6
  src/library/sensfusion6.cpp
)

add_library(mellinger_controller
  src/library/mellinger_controller.cpp
)

add_library(internal_model_controller
  src/library/internal_model_controller.cpp
)

target_link_libraries(lee_position_controller ${catkin_LIBRARIES})
add_dependencies(lee_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(position_controller ${catkin_LIBRARIES})
add_dependencies(position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(mpc_controller ${catkin_LIBRARIES})
add_dependencies(mpc_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(mellinger_controller ${catkin_LIBRARIES})
add_dependencies(mellinger_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(internal_model_controller ${catkin_LIBRARIES})
add_dependencies(internal_model_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(roll_pitch_yawrate_thrust_controller_crazyflie ${catkin_LIBRARIES})
add_dependencies(roll_pitch_yawrate_thrust_controller_crazyflie ${catkin_EXPORTED_TARGETS})

target_link_libraries(crazyflie_onboard_controller ${catkin_LIBRARIES})
add_dependencies(crazyflie_onboard_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(crazyflie_complementary_filter ${catkin_LIBRARIES})
add_dependencies(crazyflie_complementary_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(eigen_helpers ${catkin_LIBRARIES})
add_dependencies(eigen_helpers ${catkin_EXPORTED_TARGETS})

target_link_libraries(sensfusion6 ${catkin_LIBRARIES})
add_dependencies(sensfusion6 ${catkin_EXPORTED_TARGETS})

add_executable(lee_position_controller_node src/nodes/lee_position_controller_node.cpp)
add_dependencies(lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lee_position_controller_node
  lee_position_controller ${catkin_LIBRARIES})

add_executable(position_controller_node src/nodes/position_controller_node.cpp)
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_node
  position_controller crazyflie_complementary_filter crazyflie_onboard_controller
  sensfusion6 mellinger_controller internal_model_controller ${catkin_LIBRARIES})

add_executable(position_controller_light src/nodes/position_controller_light.cpp)
add_dependencies(position_controller_light ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_light
  mpc_controller position_controller crazyflie_complementary_filter crazyflie_onboard_controller
  sensfusion6 mellinger_controller internal_model_controller ${catkin_LIBRARIES})

add_executable(swarm_controller src/nodes/swarm_controller.cpp)
add_dependencies(swarm_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_controller
  mpc_controller position_controller crazyflie_complementary_filter crazyflie_onboard_controller
  sensfusion6 mellinger_controller internal_model_controller ${catkin_LIBRARIES})

add_executable(swarm_state_estimator src/nodes/swarm_state_estimator.cpp)
add_dependencies(swarm_state_estimator ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_state_estimator
  ${catkin_LIBRARIES})

add_executable(relative_distance_controller src/nodes/relative_distance_controller.cpp)
add_dependencies(relative_distance_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(relative_distance_controller
  eigen_helpers
  ${catkin_LIBRARIES})

add_executable(distance_measurement_sim src/nodes/distance_measurement_sim.cpp)
add_dependencies(distance_measurement_sim ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_measurement_sim
  ${catkin_LIBRARIES})

add_executable(playback_node src/nodes/playback_node.cpp)
add_dependencies(playback_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(playback_node
  mpc_controller crazyflie_complementary_filter crazyflie_onboard_controller
  sensfusion6 mellinger_controller internal_model_controller ${catkin_LIBRARIES})

add_executable(roll_pitch_yawrate_thrust_controller_node
  src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(roll_pitch_yawrate_thrust_controller_node
  roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})

add_executable(roll_pitch_yawrate_thrust_crazyflie_node
  src/nodes/roll_pitch_yawrate_thrust_crazyflie_node.cpp)
add_dependencies(roll_pitch_yawrate_thrust_crazyflie_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(roll_pitch_yawrate_thrust_crazyflie_node
  roll_pitch_yawrate_thrust_controller_crazyflie ${catkin_LIBRARIES})

install(TARGETS lee_position_controller position_controller crazyflie_onboard_controller
  roll_pitch_yawrate_thrust_controller mellinger_controller internal_model_controller
  roll_pitch_yawrate_thrust_controller_crazyflie
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lee_position_controller_node position_controller_node position_controller_light swarm_controller swarm_state_estimator distance_measurement_sim playback_node roll_pitch_yawrate_thrust_controller_node
  roll_pitch_yawrate_thrust_crazyflie_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
