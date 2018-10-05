cmake_minimum_required(VERSION 2.8.3)
project(openhand_node)


find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()


## Generate services in the 'srv' folder
add_service_files(
   FILES
   MoveServos.srv
   OperatingMode.srv
   ReadServos.srv
   HoldServos.srv
   ReadCurrent.srv
   ReadTemperature.srv
   ReadLoad.srv
   TorqueServos.srv
)



## Generate added messages and services with any dependencies listed here
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
