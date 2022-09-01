cmake_minimum_required(VERSION 2.8.3)
project(moveit_config_a0912)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp)

#include_directories(${catkin_INCLUDE_DIRS})
#include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
#add_executable(robot_model_and_robot_state_tutorial _scripts/#robot_model_and_robot_state_tutorial.cpp)
#target_link_libraries(robot_model_and_robot_state_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




